#ifndef __MCAL_UART_CFG_H__
#define __MCAL_UART_CFG_H__

/**
 * @file    Mcal_Uart_Cfg.h
 * @brief   USART1 link to the Linux board: pins, rate, buffer sizes.
 * @author  D0cC
 * @date    2026-10-14
 * @version V1.2.0
 *
 * @note    Every value the driver needs but that is a board or system decision
 *          lives here, so that changing the link rate or moving a pin is an
 *          edit to this file and nothing else.
 *
 * @note    Two receive implementations live in Mcal_Uart.c and
 *          UART_LINK_RX_USE_DMA picks between them. The polled one is the
 *          fallback and stays untouched: it is known to work on this board.
 *          The DMA one follows the vendor bootloader -- DMA moves the bytes and
 *          the USART idle interrupt marks the end of a frame -- which is the
 *          arrangement this board has used to receive firmware all along.
 *
 * @note    Nothing in this file may be read by the application layer. The
 *          application sees only the calls in Mcal_Uart.h.
 */

#include <stdint.h>
#include "n32h7xx.h"
#include "n32h7xx_rcc.h"
#include "n32h7xx_gpio.h"
#include "n32h7xx_usart.h"

/* 1. The peripheral and its pins.
 *
 *    USART1 is the link and the console at once: PA9 (TX) / PA10 (RX),
 *    alternate functions 7 and 5. Logs and protocol frames share the wire, and
 *    the Linux side tells them apart by the frame header, since a log line is
 *    ASCII and every protocol frame starts with the AA 55 magic. That is why
 *    there is no prefix, no second port and no framing of the log stream.
 *
 *    The clock gate names carry the core: this image is built for the M7, so
 *    every enable macro is the M7 one. Getting that wrong does not fail to
 *    compile -- it silently enables the other core's gate and the peripheral
 *    stays dark, which is the failure this comment exists to prevent.
 */
#define UART_LINK_USARTx                USART1
#define UART_LINK_CLK                   (RCC_APB1_PERIPHEN_M7_USART1)
#define UART_LINK_CLK_CMD               RCC_EnableAPB1PeriphClk3

#define UART_LINK_GPIO_CLK              (RCC_AHB5_PERIPHEN_M7_GPIOA)
#define UART_LINK_GPIO_CLK_CMD          RCC_EnableAHB5PeriphClk1

#define UART_LINK_TX_PORT               GPIOA
#define UART_LINK_TX_PIN                GPIO_PIN_9
#define UART_LINK_TX_AF                 ((uint8_t)0x07)

#define UART_LINK_RX_PORT               GPIOA
#define UART_LINK_RX_PIN                GPIO_PIN_10
#define UART_LINK_RX_AF                 ((uint8_t)0x05)

/* 2. Line format. The frames are length-delimited, so the receiver never
 *    relies on an inter-character gap; the rate only has to match the Linux
 *    side. Kept as a macro because the rate is the first thing to change when
 *    the link is characterised, and it is used nowhere else. */
#define UART_LINK_BAUDRATE              (115200u)

/* 3. Buffer sizes.
 *
 *      rx -- the bytes taken off the USART and not yet handed to the
 *            application. The largest protocol frame is 1033 bytes (9 + 1024)
 *            and the link layer above holds a frame of its own, so this only
 *            has to carry a burst plus whatever is waiting to be read.
 *
 *      tx -- one whole frame. The longest is 1033 and 1088 leaves room for the
 *            two bytes fputc() adds when a log line ends in a bare newline.
 *
 *    The USART's own hardware FIFO is 8 deep; it is enabled by the driver so
 *    that a task holding the main loop past a byte time does not lose the next
 *    byte outright.
 */
#define UART_LINK_RX_FIFO_SIZE          (1152u)
#define UART_LINK_TX_BUF_SIZE           (1088u)

/*    The largest single log line the console assembles before flushing it.
 *    160 bytes covers every line this firmware prints; anything longer goes as
 *    two lines rather than being refused. */
#define UART_LINK_LOG_LINE_SIZE         (160u)

/* 4. How long a blocking wait may spin before giving up.
 *
 *    At 115200 baud one byte takes about 87us, so a healthy wait for the
 *    transmitter is well under a hundred iterations. A million is thousands of
 *    times that: long enough that it cannot expire while the link is working,
 *    short enough that a link that has stopped cannot freeze the firmware.
 *    Every wait in the driver is bounded for this reason. */
#define UART_LINK_TX_SPIN_LIMIT         (1000000u)

/* How long a blocking wait inside an interrupt may spin.
 *
 *    A different number from above on purpose. The transmit waits run in the
 *    main loop, where spinning only costs time. This one runs in the receive
 *    interrupt, where spinning costs the 1 ms tick and the sampling interrupt
 *    their punctuality -- at 600 MHz a million iterations is tens of
 *    milliseconds, which is longer than the sampling period. Waiting for a DMA
 *    channel to acknowledge a disable takes a handful of bus cycles, so a few
 *    hundred iterations is already thousands of times what the hardware needs.
 *
 *    Exceeding it is not fatal by design: the caller skips the re-arm rather
 *    than writing registers that would not take. */
#define UART_LINK_ISR_SPIN_LIMIT        (500u)

/* 5. Receive implementation.
 *
 *    0 -- polled. The main loop reads the data register whenever the receive
 *         flag is set. Known to work; needs the main loop to come round faster
 *         than a byte time, so Mcal_Uart_Poll() belongs in the idle loop.
 *
 *    1 -- DMA plus the USART idle interrupt. The channel moves the bytes on its
 *         own and the interrupt marks the end of a frame, so neither the main
 *         loop's speed nor this driver's polling matters. This is the vendor
 *         bootloader's arrangement.
 *
 *    Switching back is this one number. Nothing else has to change: the calls
 *    in Mcal_Uart.h are the same in both modes.
 */
#define UART_LINK_RX_USE_DMA            (1)

#if (UART_LINK_RX_USE_DMA != 0)

/* The DMA target, in the 4KB past the linker's AXI SRAM region. It has to sit
 * in memory the MPU has marked non-cacheable -- the D-cache is on and AXI SRAM
 * is write-back cacheable, so a DMA that writes here while the CPU reads its
 * own cached copy would produce data that is silently wrong rather than absent.
 * 0x2401F000 is free: the scatter file stops at 0x2401EFFF and the OTA flag
 * owns 0x2401FF00. */
#define UART_LINK_DMA_BUF_BASE          (0x2401F000u)
#define UART_LINK_DMA_BUF_SIZE          (1152u)     /* >= largest frame, 1033 */

#define UART_LINK_DMAx                  DMA1
#define UART_LINK_DMAMUXx               DMAMUX1_ID

/* The two gates are in different registers and are enabled by different calls:
 * DMA1's bit lives in AHB1EN3 and DMAMUX1's in AHB1EN1. Using the Clk1 call for
 * both -- which this file did -- enables DMAMUX1 twice and leaves DMA1 dark, and
 * the map shows it: RCC_EnableAHB1PeriphClk3 is removed by the linker because
 * nothing references it. It works only because the bootloader has already opened
 * DMA1's gate and its reset leaves the gate alone, so the application inherits
 * it. Started any other way -- a debugger load, a plain reset into the
 * application -- DMA_ChannelInit() sees CFG.EN clear, returns an error, and the
 * driver's guard returns before configuring anything, including the idle
 * interrupt. */
#define UART_LINK_DMA_CLK               (RCC_AHB1_PERIPHEN_M7_DMA1)
#define UART_LINK_DMA_CLK_CMD           RCC_EnableAHB1PeriphClk3
#define UART_LINK_DMAMUX_CLK            (RCC_AHB1_PERIPHEN_M7_DMAMUX1)
#define UART_LINK_DMAMUX_CLK_CMD        RCC_EnableAHB1PeriphClk1

/* Channel 0, which is what the vendor bootloader uses for this same port and
 * request. A DMAMUX request is not tied to a channel -- the mapping is chosen
 * here -- so any channel would do; this one is chosen because the bootloader has
 * already proved it works on this board, and nothing else in the firmware uses
 * DMA at all now that transmit is polled. */
#define UART_LINK_DMA_RX_CHANNEL        DMA_CHANNEL_0
#define UART_LINK_DMA_RX_DMAMUX_CH      DMAMUX_CHANNEL_0
#define UART_LINK_DMA_RX_REQUEST        DMAMUX1_REQUEST_USART1_RX
#define UART_LINK_DMA_RX_IRQn           USART1_IRQn

/* Below the sample tick on purpose: this handler must never delay the tick, and
 * it must never be delayed by it. 2 is what the working firmware used. */
#define UART_LINK_RX_IRQ_PRIORITY       (2)

/* Handshaking interface number for USART1's receive request. There is no table
 * in the manual; the transmit side was measured at 0 on this part and the
 * bootloader writes nothing else for receive, so 0 is what the bootloader's
 * configuration amounts to. */
#define UART_LINK_RX_HS_IF              (0u)

/* Data register offset inside the USART block. */
#define UART_LINK_DR_OFFSET             (0x10u)
#define UART_LINK_DR_ADDR               ((uint32_t)((uint32_t)UART_LINK_USARTx + \
                                          (uint32_t)UART_LINK_DR_OFFSET))

#endif /* UART_LINK_RX_USE_DMA */

#endif /* __MCAL_UART_CFG_H__ */
