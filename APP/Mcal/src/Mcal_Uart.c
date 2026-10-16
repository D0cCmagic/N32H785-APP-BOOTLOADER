/**
 * @file    Mcal_Uart.c
 * @brief   USART1 byte pipe to the Linux board: two receive paths, one API.
 *
 * @note    UART_LINK_RX_USE_DMA in Mcal_Uart_Cfg.h picks which receive path is
 *          built:
 *
 *            0 -- polled. Mcal_Uart_Poll() reads the data register for as long
 *                 as the receive flag is set and appends what it finds to the
 *                 burst buffer. This is the fallback: known to work on this
 *                 board, and left alone.
 *
 *            1 -- DMA plus the USART idle interrupt. The channel moves the bytes
 *                 on its own and the idle interrupt marks the end of a burst, so
 *                 neither the main loop's speed nor polling matters. This is the
 *                 arrangement the vendor bootloader uses on this same port.
 *
 *          Both paths end in the same three published names -- flag, length,
 *          buffer -- so switching between them is the one number in the
 *          configuration file and nothing above the driver changes.
 *
 * @note    Transmit is polled in both modes: bytes go into the data register
 *          when it has room. It is the simpler of the two and it works.
 */

#include "Mcal_Uart.h"
#include "Mcal_Uart_Cfg.h"
#include "n32h7xx_dma.h"        /* DMA_ChInitType and the channel enums */
#include "n32h7xx_dmamux.h"     /* DMAMUX_SetRequestID and the request ids */
#include "misc.h"
#include <string.h>       /* memcpy, for the burst hand-off */

/* The burst buffer, and the names the application reads it through.
 *
 * The burst always starts at [0]: the idle interrupt restarts the DMA at the top
 * of its own buffer for every burst, so the driver does the same. There is no
 * head index and no wraparound -- just one length.
 *
 * One burst is held at a time, and the application reads it in place rather than
 * taking it out. A burst that arrives before the previous one has been consumed
 * is not delivered at all: its byte count goes to Mcal_Uart_RxDrop instead of
 * being written over data that is still being read. For a request/response link
 * that window never fills, and the counter is there to say so if it ever does.
 *
 * The buffer itself is not const -- the receiver fills it -- but the application
 * is only ever given a read-only view, so the pointer it sees is const. */
static uint8_t Uart_RxBuf[UART_LINK_RX_FIFO_SIZE];

volatile uint8_t  Mcal_Uart_RxFlag = 0u;
volatile uint16_t Mcal_Uart_RxLen  = 0u;
volatile uint32_t Mcal_Uart_RxDrop = 0u;
const uint8_t    *Mcal_Uart_RxBuf  = Uart_RxBuf;
static uint8_t           Uart_InitDone;

#if (UART_LINK_RX_USE_DMA != 0)

/* ---- DMA receive path --------------------------------------------------- */

/* The channel's target. Fixed in the MPU's non-cacheable window: the D-cache is
 * on and AXI SRAM is write-back cacheable, so without that the CPU would read a
 * stale cached copy of what the DMA just wrote -- wrong data, no error. */
static uint8_t Uart_DmaBuf[UART_LINK_DMA_BUF_SIZE]
    __attribute__((at(UART_LINK_DMA_BUF_BASE)));

/* Which channel-enable bits this driver wants set.
 *
 * The mask is kept here rather than read back from CHEN because a read-modify-
 * write of CHEN cannot be trusted: the hardware clears a channel's enable bit on
 * its own when a block completes and when the channel is disabled, so a value
 * read a moment ago may already be stale. Keeping the intent means the value
 * written is always the value wanted. */
static volatile uint32_t Uart_ChEn;

static void Mcal_Uart_ConfigMpu(void);
static void Mcal_Uart_DmaRxInit(void);
static void Mcal_Uart_ChEnCommit(DMA_ChNumType Ch);
static void Mcal_Uart_ChannelEnable(DMA_ChNumType Ch);
static void Mcal_Uart_ChannelDisable(DMA_ChNumType Ch);

/**
 * @name    Mcal_Uart_ConfigMpu
 * @brief   Make the buffer block non-cacheable so the CPU and the DMA agree.
 * @param   None
 * @retval  None
 * @note    Region 0 is free: main.c's MPU_Config() sets the background map and
 *          leaves the regions unused.
 */
static void Mcal_Uart_ConfigMpu(void)
{
    MPU_Region_InitType Region;

    MPU_Disable();

    Region.Enable           = MPU_REGION_ENABLE;
    Region.Number           = MPU_REGION_NUMBER0;
    Region.BaseAddress      = UART_LINK_DMA_BUF_BASE;
    Region.Size             = MPU_REGION_SIZE_4KB;
    Region.SubRegionDisable = 0x00u;
    Region.TypeExtField     = MPU_TEX_LEVEL0;
    Region.AccessPermission = MPU_REGION_FULL_ACCESS;
    Region.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    Region.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    Region.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    Region.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    MPU_ConfigRegion(&Region);
    MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @name    Mcal_Uart_ChEnCommit
 * @brief   Write the whole channel-enable mask with WEN set for every channel.
 * @param   None
 * @retval  None
 * @note    CHEN bits [7:0] enable channels 0..7 and bits [15:8] are their write
 *          enables: a channel bit is only adopted when its own WEN bit is set in
 *          the same write. The vendor's DMA_ChannelCmd() sets no WEN at all,
 *          which is why it enables nothing on this part -- and it writes a
 *          single-channel value into the whole register, so calling it for one
 *          channel would switch the other one off. Both problems are avoided by
 *          writing the complete mask here.
 */
static void Mcal_Uart_ChEnCommit(DMA_ChNumType Ch)
{
    /* Write-enable only the channel being changed. Setting every WEN bit -- which
     * this did -- makes the write authoritative for all eight channels, so the
     * ones whose enable bit is 0 in the mask get switched off. Only the target
     * channel's WEN is set, so nothing else is adopted. */
    UART_LINK_DMAx->CHEN = (Uart_ChEn & 0xFFu) | (1u << ((uint32_t)Ch + 8u));
}

static void Mcal_Uart_ChannelEnable(DMA_ChNumType Ch)
{
    Uart_ChEn |= (1u << (uint32_t)Ch);
    Mcal_Uart_ChEnCommit(Ch);
}

/**
 * @name    Mcal_Uart_ChannelDisable
 * @brief   Disable one DMA channel, with a bound on the wait.
 * @param   Ch: channel number.
 * @retval  None
 * @note    The vendor's DMA_ChannelCmd(DISABLE) waits on the enable bit with no
 *          bound. That wait does not always end, and when it does not the whole
 *          firmware stops there: no console, no LEDs, nothing to say why,
 *          because the console is driven by the same core.
 * @note    Reading the remaining count and writing the destination address are
 *          only possible while the channel is disabled, so this has to work --
 *          but losing the wait is survivable and losing the firmware is not.
 */
static void Mcal_Uart_ChannelDisable(DMA_ChNumType Ch)
{
    uint32_t Spin = 0u;

    Uart_ChEn &= ~(1u << (uint32_t)Ch);
    Mcal_Uart_ChEnCommit(Ch);

    while ((UART_LINK_DMAx->CHEN & (1u << (uint32_t)Ch)) != 0u)
    {
        Spin++;
        if (Spin >= UART_LINK_ISR_SPIN_LIMIT)
        {
            break;
        }
    }
}

/**
 * @name    USART1_IRQHandler
 * @brief   End of a received burst: publish it, then re-arm for the next one.
 * @param   None
 * @retval  None
 * @note    THIS FUNCTION MUST EXIST. The startup file vectors this slot at a
 *          weak symbol whose body is an endless loop, so if it is missing the
 *          first idle interrupt stops the CPU there for good -- no console
 *          output, no LED, nothing to say why. After a build, confirm the symbol
 *          comes from this file rather than from startup_*.o in the map:
 *
 *            Select-String -Path MDK-ARM\N32H78x\Listings\LWIP_PING.map `
 *                          -Pattern 'USART1_IRQHandler'
 *
 * @note    The idle flag is cleared by reading the status register and then the
 *          data register, in that order. The hardware defines the sequence and
 *          writing the register does not clear it.
 * @note    USART_FLAG_IDLEF, not USART_INT_IDLEF: the INT form carries the
 *          enable bit as well, and USART_GetIntStatus() ANDs its argument with
 *          CTRL1 -- so it answers "the interrupt is enabled", which became true
 *          when it was turned on, and never says whether the line went idle.
 * @note    The channel is stopped before the count is read: CHxCTRL is only
 *          writable while the channel is disabled, which the re-arm needs
 *          anyway, and with it stopped the count cannot move under us.
 * @note    DMA_GetTransferredNumber() returns what was MOVED, not what is left.
 *          That is what the SDK's name and its return description say, and it is
 *          what this board measures -- ten after a ten-byte frame. Do not
 *          "correct" it with a subtraction.
 * @note    The idle flag is what marks the end of a burst, so it is also what
 *          decides that a burst is complete. There is no other completion
 *          signal, and nothing here prints or parses.
 */
void USART1_IRQHandler(void)
{
    uint32_t Moved;

    if (USART_GetFlagStatus(UART_LINK_USARTx, USART_FLAG_IDLEF) == RESET)
    {
        return;
    }

    /* 清空闲标志：读 STS，再读 DAT */
    (void)UART_LINK_USARTx->STS;
    (void)UART_LINK_USARTx->DAT;

    Mcal_Uart_ChannelDisable(UART_LINK_DMA_RX_CHANNEL);

    Moved = (uint32_t)DMA_GetTransferredNumber(UART_LINK_DMAx,
                                               UART_LINK_DMA_RX_CHANNEL);
    if (Moved > (uint32_t)UART_LINK_RX_FIFO_SIZE)
    {
        Moved = (uint32_t)UART_LINK_RX_FIFO_SIZE;
    }

    /* Publish, but only when the previous burst has been consumed. The
     * application clears the flag while it is working on the buffer, so a burst
     * arriving during that window cannot be delivered -- it is counted instead
     * of being written over data that is still being read. A request/response
     * link does not fill that window; if that ever changes this counter is what
     * says so. */
    if (Mcal_Uart_RxFlag != 0u)
    {
        Mcal_Uart_RxDrop += Moved;
    }
    else if (Moved != 0u)
    {
        memcpy(Uart_RxBuf, Uart_DmaBuf, (size_t)Moved);
        Mcal_Uart_RxLen  = (uint16_t)Moved;
        Mcal_Uart_RxFlag = 1u;          /* 最后置，长度和数据先就位 */
    }
    else
    {
        /* Nothing arrived. */
    }

    /* Put the channel back at the top of its buffer for the next burst. These
     * writes only take effect while the channel is disabled. */
    DMA_ClearChannelEventStatus(UART_LINK_DMAx, UART_LINK_DMA_RX_CHANNEL,
        (uint32_t)((uint32_t)DMA_CH_EVENT_TRANSFER_COMPLETE |
                   (uint32_t)DMA_CH_EVENT_ERROR));
    DMA_SetChannelDestinationAddress(UART_LINK_DMAx, UART_LINK_DMA_RX_CHANNEL,
                                     (uint32_t *)Uart_DmaBuf);
    DMA_SetChannelBlockSize(UART_LINK_DMAx, UART_LINK_DMA_RX_CHANNEL,
                            UART_LINK_DMA_BUF_SIZE);
    Mcal_Uart_ChannelEnable(UART_LINK_DMA_RX_CHANNEL);
}

/**
 * @name    Mcal_Uart_DmaRxInit
 * @brief   Set up the receive channel and the idle interrupt.
 * @param   None
 * @retval  None
 * @note    Single block, not circular: the idle interrupt stops and restarts the
 *          channel for every frame, so the remaining count always means "bytes
 *          moved this frame" and the buffer always starts at the top. Channel 0
 *          is the one the vendor bootloader uses for this port and request.
 * @note    The idle interrupt is enabled last, after the channel is armed and
 *          the USART is on, so an interrupt cannot arrive before there is
 *          anywhere for its handler to look.
 */
static void Mcal_Uart_DmaRxInit(void)
{
    /* Zeroed on purpose. DMA_ChannelInit() writes the whole 64-bit CTRL word out
     * of this union, and the descriptor has unnamed padding over bits the
     * hardware gives meaning to -- the linked-list enables among them. Neither
     * DMA_ChannelStructInit() nor this function assigns those, so an
     * uninitialised local carries whatever the stack held into the register. The
     * vendor bootloader and every vendor example write = {0} here. */
    DMA_ChInitType DmaChInit = {0};

    UART_LINK_DMA_CLK_CMD(UART_LINK_DMA_CLK, ENABLE);
    UART_LINK_DMAMUX_CLK_CMD(UART_LINK_DMAMUX_CLK, ENABLE);

    Mcal_Uart_ConfigMpu();

    Uart_ChEn = 0u;
    Mcal_Uart_ChEnCommit(UART_LINK_DMA_RX_CHANNEL);

    /* The controller must be enabled before a channel is programmed. */
    DMA_ControllerCmd(UART_LINK_DMAx, ENABLE);

    DMA_ChannelStructInit(&DmaChInit);
    DmaChInit.IntEn            = 0x1u;
    DmaChInit.SrcAddr          = UART_LINK_DR_ADDR;
    DmaChInit.DstAddr          = (uint32_t)Uart_DmaBuf;
    DmaChInit.SrcTfrWidth      = DMA_CH_TRANSFER_WIDTH_8;
    DmaChInit.DstTfrWidth      = DMA_CH_TRANSFER_WIDTH_8;
    DmaChInit.SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
    DmaChInit.DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
    DmaChInit.SrcBurstLen      = DMA_CH_BURST_LENGTH_1;
    DmaChInit.DstBurstLen      = DMA_CH_BURST_LENGTH_1;
    DmaChInit.TfrTypeFlowCtrl  = (uint64_t)DMA_CH_TRANSFER_FLOW_P2M_DMA;
    DmaChInit.TfrType          = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
    DmaChInit.ChannelPriority  = DMA_CH_PRIORITY_7;
    DmaChInit.SrcHandshaking   = DMA_CH_SRC_HANDSHAKING_HARDWARE;
    DmaChInit.SrcHsInterface   = DMA_CH_HARDWARE_HANDSHAKING_IF_0;
    /* No destination handshaking: the destination is memory and memory never
     * requests. Leaving it at the struct default (software) is what the
     * bootloader's configuration amounts to. */
    DmaChInit.BlkTfrSize       = UART_LINK_DMA_BUF_SIZE;

    if (DMA_ChannelInit(UART_LINK_DMAx, &DmaChInit, UART_LINK_DMA_RX_CHANNEL) !=
        DMA_CH_STS_OK)
    {
        return;
    }

    /* The block size in the struct does not reach the register; write it again
     * through the call that only touches the CTRL block-size field. Without it
     * the channel moves the SDK's default of two bytes and stops. */
    DMA_SetChannelBlockSize(UART_LINK_DMAx, UART_LINK_DMA_RX_CHANNEL,
                            UART_LINK_DMA_BUF_SIZE);

    DMA_ChannelEventCmd(UART_LINK_DMAx, UART_LINK_DMA_RX_CHANNEL,
                        (uint32_t)DMA_CH_EVENT_TRANSFER_COMPLETE, ENABLE);
    DMAMUX_SetRequestID(UART_LINK_DMAMUXx, UART_LINK_DMA_RX_DMAMUX_CH,
                        UART_LINK_DMA_RX_REQUEST);

    /* Force hardware handshaking and write the interface number the SDK never
     * writes: HSSELSRC/HSSELDST must be clear and SRCPER carries the number. */
    UART_LINK_DMAx->CH[UART_LINK_DMA_RX_CHANNEL].CFG &=
        ~((uint64_t)DMA_CHNCFG_HSSELDST | (uint64_t)DMA_CHNCFG_HSSELSRC);
    UART_LINK_DMAx->CH[UART_LINK_DMA_RX_CHANNEL].CFG &=
        ~((uint64_t)DMA_CHNCFG_SRCPER);
    UART_LINK_DMAx->CH[UART_LINK_DMA_RX_CHANNEL].CFG |=
        (((uint64_t)UART_LINK_RX_HS_IF) << 39);

    USART_EnableDMA(UART_LINK_USARTx, USART_DMAREQ_RX, ENABLE);

    /* The bootloader leaves DMA1 channels enabled in the NVIC and its vector
     * slots are the startup file's endless loop. This driver polls nothing on
     * DMA and takes no DMA interrupt, so they are switched off outright. */
    NVIC_DisableIRQ(DMA1_Channel0_IRQn);
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel0_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);

    Mcal_Uart_ChannelEnable(UART_LINK_DMA_RX_CHANNEL);

    USART_ConfigInt(UART_LINK_USARTx, USART_INT_IDLEF, ENABLE);
    NVIC_SetPriority(UART_LINK_DMA_RX_IRQn, UART_LINK_RX_IRQ_PRIORITY);
    NVIC_EnableIRQ(UART_LINK_DMA_RX_IRQn);
}

#endif /* UART_LINK_RX_USE_DMA */

/**
 * @name    Mcal_Uart_Init
 * @brief   Bring USART1 up for polled use.
 * @param   None
 * @retval  None
 * @note    8N1 at UART_LINK_BAUDRATE, PA9/PA10. The receive path is chosen by
 *          UART_LINK_RX_USE_DMA; transmit is polled either way.
 */
void Mcal_Uart_Init(void)
{
    GPIO_InitType  GpioInit;
    USART_InitType UsartInit;

    if (Uart_InitDone != 0u)
    {
        return;
    }
    Uart_InitDone = 1u;

    UART_LINK_CLK_CMD(UART_LINK_CLK, ENABLE);
    UART_LINK_GPIO_CLK_CMD(UART_LINK_GPIO_CLK, ENABLE);

    GPIO_InitStruct(&GpioInit);
    GpioInit.Pin            = UART_LINK_TX_PIN;
    GpioInit.GPIO_Mode      = GPIO_MODE_AF_PP;
    GpioInit.GPIO_Pull      = GPIO_PULL_UP;
    GpioInit.GPIO_Alternate = UART_LINK_TX_AF;
    GPIO_InitPeripheral(UART_LINK_TX_PORT, &GpioInit);
    GpioInit.Pin            = UART_LINK_RX_PIN;
    GpioInit.GPIO_Mode      = GPIO_MODE_AF_PP;
    GpioInit.GPIO_Pull      = GPIO_PULL_UP;
    GpioInit.GPIO_Alternate = UART_LINK_RX_AF;
    GPIO_InitPeripheral(UART_LINK_RX_PORT, &GpioInit);

    USART_StructInit(&UsartInit);
    UsartInit.BaudRate            = UART_LINK_BAUDRATE;
    UsartInit.WordLength          = USART_WL_8B;
    UsartInit.StopBits            = USART_STPB_1;
    UsartInit.Parity              = USART_PE_NO;
    UsartInit.HardwareFlowControl = USART_HFCTRL_NONE;
    UsartInit.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_Init(UART_LINK_USARTx, &UsartInit);

    Mcal_Uart_RxFlag = 0u;
    Mcal_Uart_RxLen  = 0u;
    Mcal_Uart_RxDrop = 0u;

    USART_Enable(UART_LINK_USARTx, ENABLE);

#if (UART_LINK_RX_USE_DMA != 0)
    /* After the USART is on, so the request line is live before the channel is
     * armed and the idle interrupt is enabled. */
    Mcal_Uart_DmaRxInit();
#endif
}

/**
 * @name    Mcal_Uart_Poll
 * @brief   Polled receive mode: take whatever the receiver has.
 * @param   None
 * @retval  None
 * @note    Compiled only when UART_LINK_RX_USE_DMA is 0. In DMA mode the channel
 *          and the idle interrupt do the work and this does not exist at all.
 * @note    There is no idle interrupt here to say where a burst ends, so the
 *          end is detected the only way a polled loop can: when the receive
 *          flag has gone clear, everything that had arrived has been read. That
 *          makes the boundary best-effort -- the flag can go clear inside a
 *          frame if the far end pauses -- which is exactly why the DMA path
 *          exists.
 */
static void Mcal_Uart_PollRx(void)
{
    uint32_t New = 0u;
    uint32_t Put;

    /* Left over from the previous burst, which the application did not take:
     * it is about to be overwritten, so count it rather than lose it quietly. */
    if (Mcal_Uart_RxFlag != 0u)
    {
        Mcal_Uart_RxDrop += (uint32_t)Mcal_Uart_RxLen;
    }

    while (USART_GetFlagStatus(UART_LINK_USARTx, USART_FLAG_RXDNE) != RESET)
    {
        if (New < (uint32_t)UART_LINK_RX_FIFO_SIZE)
        {
            Uart_RxBuf[New] = (uint8_t)USART_ReceiveData(UART_LINK_USARTx);
            New++;
        }
        else
        {
            (void)USART_ReceiveData(UART_LINK_USARTx);   /* 缓冲满，读走丢弃 */
            Mcal_Uart_RxDrop++;
        }
    }

    if (New != 0u)
    {
        Mcal_Uart_RxLen  = (uint16_t)New;
        Mcal_Uart_RxFlag = 1u;
    }

    /* The idle flag is not used here to delimit anything -- a polled loop
     * cannot see the instant it is set -- but it must not be left standing
     * either. Reading STS and then DAT clears it. */
    if (USART_GetFlagStatus(UART_LINK_USARTx, USART_FLAG_IDLEF) != RESET)
    {
        (void)UART_LINK_USARTx->STS;
        (void)UART_LINK_USARTx->DAT;
    }
}


#if (UART_LINK_RX_USE_DMA == 0)
/**
 * @name    Mcal_Uart_Poll
 * @brief   Polled receive mode: take whatever the receiver has.
 * @param   None
 * @retval  None
 * @note    The application calls this every slot in polled mode. In DMA mode the
 *          function does not exist -- the channel and the idle interrupt do the
 *          work -- which is why the callers guard it with UART_LINK_RX_USE_DMA.
 */
void Mcal_Uart_Poll(void)
{
    Mcal_Uart_PollRx();
}
#endif /* UART_LINK_RX_USE_DMA */

/**
 * @name    Mcal_Uart_Send
 * @brief   Write bytes straight out of the data register, with a bound.
 * @param   Data: bytes to send.
 * @param   Len: number of bytes, 1 to UART_LINK_TX_BUF_SIZE.
 * @retval  0 on success; -1 if the transmitter never made room, or the length
 *          is unusable.
 * @note    Each byte waits for TXDE with a bound, so a stopped transmitter
 *          cannot freeze the firmware.
 */
int32_t Mcal_Uart_Send(uint8_t *Data, uint32_t Len)
{
    uint32_t i;
    uint32_t Spin;

    if ((Data == (uint8_t *)0) || (Len == 0u) || (Len > UART_LINK_TX_BUF_SIZE))
    {
        return -1;
    }

    for (i = 0u; i < Len; i++)
    {
        Spin = 0u;
        while (USART_GetFlagStatus(UART_LINK_USARTx, USART_FLAG_TXDE) == RESET)
        {
            Spin++;
            if (Spin >= UART_LINK_TX_SPIN_LIMIT)
            {
                return -1;
            }
        }
        USART_SendData(UART_LINK_USARTx, Data[i]);
    }

    return 0;
}

/**
 * @name    Mcal_Uart_TxWaitDone
 * @brief   Wait, with a bound, for the last byte to leave the shift register.
 * @param   SpinLimit: poll iterations before giving up.
 * @retval  1 when idle, 0 when the bound ran out.
 * @note    For the OTA path, which reboots the part and must let the last
 *          message out.
 */
uint8_t Mcal_Uart_TxWaitDone(uint32_t SpinLimit)
{
    uint32_t Spin = 0u;

    while (USART_GetFlagStatus(UART_LINK_USARTx, USART_FLAG_TXC) == RESET)
    {
        Spin++;
        if (Spin >= SpinLimit)
        {
            return 0u;
        }
    }

    return 1u;
}



#if (UART_LINK_RX_USE_DMA != 0)



#endif /* UART_LINK_RX_USE_DMA */
