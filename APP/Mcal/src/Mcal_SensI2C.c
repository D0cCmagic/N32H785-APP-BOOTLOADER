/**
 * @file    Mcal_SensI2C.c
 * @brief   Software (bit-banged) I2C master implementation.
 * @author  D0cC
 * @date    2026-09-29
 * @version V1.2.0
 *
 * @note    Targets the N32H785 using the N32H7xx standard peripheral library.
 *          See Mcal_SensI2C.h for why this is a bit-bang master, why there are
 *          two buses, and what each public function promises -- the interface
 *          documentation lives there, not here.
 *
 *          What is here is the bit timing and the transaction layer. Every
 *          primitive below takes the pin set of the bus it is driving, so there
 *          is no "current bus" global to get out of step and one copy of the
 *          bit-bang logic serves both buses.
 */

#include "Mcal_SensI2C.h"

/* The DWT cycle counter, addressed directly rather than through the CMSIS
 * core header: this is the only place in the firmware that needs it, and the
 * register addresses are architectural and do not move between Cortex-M7
 * implementations. */
#define SENS_DEMCR                      (*(volatile uint32_t *)0xE000EDFCUL)
#define SENS_DWT_CTRL                   (*(volatile uint32_t *)0xE0001000UL)
#define SENS_DWT_CYCCNT                 (*(volatile uint32_t *)0xE0001004UL)

#define SENS_DEMCR_TRCENA               (0x01000000UL)
#define SENS_DWT_CTRL_CYCCNTENA         (0x00000001UL)

/**
 * @brief  The two pins of one bus.
 */
typedef struct
{
    GPIO_Module   *SclPort;
    uint16_t       SclPin;
    GPIO_Module   *SdaPort;
    uint16_t       SdaPin;
    __IO uint32_t *AnaenReg;
    uint32_t       AnaenClr;
} SensI2C_PinCfg_t;

/* One entry per SensI2C_Bus_t, in enum order. The pins themselves come from
 * the header so that the wiring is described in exactly one place. */
static const SensI2C_PinCfg_t SensI2C_Pin[SENS_I2C_BUS_COUNT] =
{
    { SENS_I2C0_SCL_PORT, SENS_I2C0_SCL_PIN, SENS_I2C0_SDA_PORT, SENS_I2C0_SDA_PIN,
      SENS_I2C0_ANAEN_REG, SENS_I2C0_ANAEN_CLR },
    { SENS_I2C1_SCL_PORT, SENS_I2C1_SCL_PIN, SENS_I2C1_SDA_PORT, SENS_I2C1_SDA_PIN,
      SENS_I2C1_ANAEN_REG, SENS_I2C1_ANAEN_CLR }
};

#define SENS_SCL_HIGH(cfg)              GPIO_SetBits((cfg)->SclPort, (cfg)->SclPin)
#define SENS_SCL_LOW(cfg)               GPIO_ResetBits((cfg)->SclPort, (cfg)->SclPin)
#define SENS_SDA_HIGH(cfg)              GPIO_SetBits((cfg)->SdaPort, (cfg)->SdaPin)
#define SENS_SDA_LOW(cfg)               GPIO_ResetBits((cfg)->SdaPort, (cfg)->SdaPin)

/* Both pins stay in open-drain output mode for the whole transfer. Reading
 * the input register of an output-mode pin returns the pad level, not the
 * value last driven, and that is what makes it possible to read SDA, to
 * notice a slave stretching the clock and to spot a bus shorted low. */
#define SENS_SCL_READ(cfg)              (GPIO_ReadInputDataBit((cfg)->SclPort, (cfg)->SclPin) != 0u)
#define SENS_SDA_READ(cfg)              (GPIO_ReadInputDataBit((cfg)->SdaPort, (cfg)->SdaPin) != 0u)

/* Core clock in cycles per microsecond, captured once at init */
static uint32_t SensI2C_CyclesPerUs = 0u;

static const SensI2C_PinCfg_t *SensI2C_Cfg(SensI2C_Bus_t Bus);
static void    SensI2C_DelayUs(uint32_t us);
static void    SensI2C_InitCycleCounter(void);
static int32_t SensI2C_PadCheck(const SensI2C_PinCfg_t *cfg);
static int32_t SensI2C_ReleaseClock(const SensI2C_PinCfg_t *cfg);
static int32_t SensI2C_Start(const SensI2C_PinCfg_t *cfg);
static void    SensI2C_Stop(const SensI2C_PinCfg_t *cfg);
static int32_t SensI2C_PutByte(const SensI2C_PinCfg_t *cfg, uint8_t Value);
static int32_t SensI2C_PutAddr(const SensI2C_PinCfg_t *cfg, uint8_t DevAddr7, uint8_t Read);
static int32_t SensI2C_StartWrite(const SensI2C_PinCfg_t *cfg, uint8_t DevAddr7, uint8_t Reg);
static int32_t SensI2C_GetByte(const SensI2C_PinCfg_t *cfg, uint8_t Ack, uint8_t *pValue);

/**
 * @name    SensI2C_Cfg
 * @brief   Map a bus index to its pin set.
 * @param   Bus: bus index from the caller.
 * @retval  Pointer to the pin set, or NULL when the index is out of range.
 * @note    The single place a bad bus index is caught, so no public entry
 *          point can index the table out of bounds.
 */
static const SensI2C_PinCfg_t *SensI2C_Cfg(SensI2C_Bus_t Bus)
{
    if ((uint32_t)Bus >= (uint32_t)SENS_I2C_BUS_COUNT)
    {
        return 0;
    }

    return &SensI2C_Pin[Bus];
}

/**
 * @name    SensI2C_PadCheck
 * @brief   Drive each line of a bus low, then let it go, checking both steps.
 * @param   cfg: pins of the bus being checked.
 * @retval  SENS_I2C_OK when both lines did both things, else the bus error.
 * @note    Called from the bus initialisation, before anything is sent and
 *          before any device is known to be present, because a pad that cannot
 *          carry a bus fails in a way the transfer layer can only report as a
 *          timeout on the first address byte -- and a board with such a pad
 *          looks exactly like a board with a dead sensor. PC6 on this board is
 *          one: at idle, with nothing connected to it, it still reads low.
 * @note    The delays are deliberately generous: the release step is reading a
 *          line that only the weak internal pull-up is raising.
 */
static int32_t SensI2C_PadCheck(const SensI2C_PinCfg_t *cfg)
{
    SENS_SCL_LOW(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US * 10u);
    if (SENS_SCL_READ(cfg))
    {
        SENS_SCL_HIGH(cfg);
        return SENS_I2C_ERR_BUS_STUCK;      /* cannot pull its line down */
    }

    SENS_SCL_HIGH(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US * 10u);
    if (SENS_SCL_READ(cfg) == 0u)
    {
        return SENS_I2C_ERR_BUS_STUCK;      /* cannot let its line go     */
    }

    SENS_SDA_LOW(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US * 10u);
    if (SENS_SDA_READ(cfg))
    {
        SENS_SDA_HIGH(cfg);
        return SENS_I2C_ERR_BUS_STUCK;
    }

    SENS_SDA_HIGH(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US * 10u);
    if (SENS_SDA_READ(cfg) == 0u)
    {
        return SENS_I2C_ERR_BUS_STUCK;
    }

    return SENS_I2C_OK;
}

int32_t Mcal_SensI2C_Init(SensI2C_Bus_t Bus)
{
    const SensI2C_PinCfg_t *cfg = SensI2C_Cfg(Bus);
    GPIO_InitType gpio_init;

    if (cfg == 0)
    {
        return SENS_I2C_ERR_PARAM;
    }

    SENS_I2C_GPIO_CLK_ENABLE();

    /* The analog-enable register sits behind the AFIO clock, so that gate has to
     * be open before the register can be written. Nothing else in this build
     * enables it: the two files that do are not in the Keil project. */
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO, ENABLE);

    /* Analog function off for any pad of this bus that has one. A pad left in
     * analog mode can neither drive nor release its line, and the bus then reads
     * SCL low at idle with SDA fine, which no transfer can get past. */
    if (cfg->AnaenClr != 0u)
    {
        *cfg->AnaenReg &= ~cfg->AnaenClr;
    }

    SENS_SCL_HIGH(cfg);
    SENS_SDA_HIGH(cfg);

    GPIO_InitStruct(&gpio_init);
    gpio_init.GPIO_Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.GPIO_Pull = GPIO_PULL_UP;

    gpio_init.Pin = cfg->SclPin;
    GPIO_InitPeripheral(cfg->SclPort, &gpio_init);

    gpio_init.Pin = cfg->SdaPin;
    GPIO_InitPeripheral(cfg->SdaPort, &gpio_init);

    SensI2C_InitCycleCounter();

    /* Prove both pads before anything is sent on them */
    if (SensI2C_PadCheck(cfg) != SENS_I2C_OK)
    {
        return SENS_I2C_ERR_BUS_STUCK;
    }

    return Mcal_SensI2C_BusRecover(Bus);
}

int32_t Mcal_SensI2C_Probe(SensI2C_Bus_t Bus, uint8_t DevAddr7)
{
    const SensI2C_PinCfg_t *cfg = SensI2C_Cfg(Bus);
    int32_t ret;

    if ((cfg == 0) || (DevAddr7 > SENS_I2C_ADDR7_MAX))
    {
        return SENS_I2C_ERR_PARAM;
    }

    ret = SensI2C_Start(cfg);
    if (ret == SENS_I2C_OK)
    {
        ret = SensI2C_PutAddr(cfg, DevAddr7, 0u);
    }

    SensI2C_Stop(cfg);
    return ret;
}

int32_t Mcal_SensI2C_WriteReg(SensI2C_Bus_t Bus, uint8_t DevAddr7, uint8_t Reg, uint8_t Value)
{
    return Mcal_SensI2C_WriteRegs(Bus, DevAddr7, Reg, &Value, 1u);
}

int32_t Mcal_SensI2C_WriteRegs(SensI2C_Bus_t Bus, uint8_t DevAddr7, uint8_t Reg,
                               const uint8_t *pBuf, uint16_t Len)
{
    const SensI2C_PinCfg_t *cfg = SensI2C_Cfg(Bus);
    uint16_t i;
    int32_t  ret;

    if ((cfg == 0) || (DevAddr7 > SENS_I2C_ADDR7_MAX) || (pBuf == 0) || (Len == 0u))
    {
        return SENS_I2C_ERR_PARAM;
    }

    ret = SensI2C_StartWrite(cfg, DevAddr7, Reg);

    if (ret != SENS_I2C_OK)
    {
        SensI2C_Stop(cfg);
        return ret;
    }

    for (i = 0u; (ret == SENS_I2C_OK) && (i < Len); i++)
    {
        ret = SensI2C_PutByte(cfg, pBuf[i]);
    }

    SensI2C_Stop(cfg);
    return ret;
}

int32_t Mcal_SensI2C_ReadReg(SensI2C_Bus_t Bus, uint8_t DevAddr7, uint8_t Reg, uint8_t *pValue)
{
    return Mcal_SensI2C_ReadRegs(Bus, DevAddr7, Reg, pValue, 1u);
}

int32_t Mcal_SensI2C_ReadRegs(SensI2C_Bus_t Bus, uint8_t DevAddr7, uint8_t Reg,
                              uint8_t *pBuf, uint16_t Len)
{
    const SensI2C_PinCfg_t *cfg = SensI2C_Cfg(Bus);
    uint16_t i;
    int32_t  ret;

    if ((cfg == 0) || (DevAddr7 > SENS_I2C_ADDR7_MAX) || (pBuf == 0) || (Len == 0u))
    {
        return SENS_I2C_ERR_PARAM;
    }

    ret = SensI2C_StartWrite(cfg, DevAddr7, Reg);

    if (ret == SENS_I2C_OK)
    {
        ret = SensI2C_Start(cfg);
    }

    if (ret == SENS_I2C_OK)
    {
        ret = SensI2C_PutAddr(cfg, DevAddr7, 1u);
    }

    if (ret != SENS_I2C_OK)
    {
        SensI2C_Stop(cfg);
        return ret;
    }

    /* Every byte but the last is acknowledged so the slave keeps counting */
    for (i = 0u; (ret == SENS_I2C_OK) && (i < Len); i++)
    {
        ret = SensI2C_GetByte(cfg, (uint8_t)((i + 1u < Len) ? 1u : 0u), &pBuf[i]);
    }

    SensI2C_Stop(cfg);
    return ret;
}

int32_t Mcal_SensI2C_Scan(SensI2C_Bus_t Bus, uint8_t *pFound, uint8_t MaxFound)
{
    const SensI2C_PinCfg_t *cfg = SensI2C_Cfg(Bus);
    uint8_t Found = 0u;
    uint8_t Addr;
    int32_t ret;

    if (cfg == 0)
    {
        return SENS_I2C_ERR_PARAM;
    }
		
    if (SENS_SDA_READ(cfg) == 0u)
    {
        return SENS_I2C_ERR_BUS_STUCK;
    }

    /* 0x00..0x07 and 0x78..0x7F are reserved, so no device can live there */
    for (Addr = 0x08u; Addr <= 0x77u; Addr++)
    {
        ret = Mcal_SensI2C_Probe(Bus, Addr);
        if (ret == SENS_I2C_OK)
        {
            if ((pFound != 0) && (Found < MaxFound))
            {
                pFound[Found] = Addr;
            }
            Found++;
        }
        else if (ret != SENS_I2C_ERR_NACK_ADDR)
        {
            /* A timeout or a stuck bus is a fault, not a missing device */
            return ret;
        }
    }

    return (int32_t)Found;
}

int32_t Mcal_SensI2C_GetLineLevels(SensI2C_Bus_t Bus, uint8_t *pSclHigh, uint8_t *pSdaHigh)
{
    const SensI2C_PinCfg_t *cfg = SensI2C_Cfg(Bus);

    if ((cfg == 0) || (pSclHigh == 0) || (pSdaHigh == 0))
    {
        return SENS_I2C_ERR_PARAM;
    }

    /* Release both lines first so that what is measured is the idle state and
     * not whatever the previous transfer left behind */
    SENS_SCL_HIGH(cfg);
    SENS_SDA_HIGH(cfg);

    *pSclHigh = (SENS_SCL_READ(cfg) != 0u) ? 1u : 0u;
    *pSdaHigh = (SENS_SDA_READ(cfg) != 0u) ? 1u : 0u;

    return SENS_I2C_OK;
}

int32_t Mcal_SensI2C_BusRecover(SensI2C_Bus_t Bus)
{
    const SensI2C_PinCfg_t *cfg = SensI2C_Cfg(Bus);
    uint8_t i;

    if (cfg == 0)
    {
        return SENS_I2C_ERR_PARAM;
    }

    SENS_SDA_HIGH(cfg);

    /* A slave that was interrupted mid-byte still holds SDA low and is
     * waiting for the clock edges that will finish its byte. Feed it up to
     * nine of them, which is enough to complete any single byte plus its
     * acknowledge, and stop as soon as it lets go */
    for (i = 0u; i < 9u; i++)
    {
        if (SENS_SDA_READ(cfg) != 0u)
        {
            break;
        }
        SENS_SCL_LOW(cfg);
        SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);
        SENS_SCL_HIGH(cfg);
        SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);
    }

    SensI2C_Stop(cfg);

    return (SENS_SDA_READ(cfg) != 0u) ? SENS_I2C_OK : SENS_I2C_ERR_BUS_STUCK;
}

/**
 * @name    SensI2C_DelayUs
 * @brief   Busy-wait a number of microseconds on the DWT cycle counter.
 * @param   us: microseconds to wait.
 * @retval  None
 * @note    Falls through immediately when the cycle counter is not running,
 *          which lets the bus run too fast rather than wedge the bit loop.
 */
static void SensI2C_DelayUs(uint32_t us)
{
    uint32_t start;
    uint32_t wait;

    if ((us == 0u) || (SensI2C_CyclesPerUs == 0u))
    {
        return;
    }

    wait  = us * SensI2C_CyclesPerUs;
    start = SENS_DWT_CYCCNT;

    while ((uint32_t)(SENS_DWT_CYCCNT - start) < wait)
    {
    }
}

/**
 * @name    SensI2C_InitCycleCounter
 * @brief   Enable the DWT cycle counter and capture the core clock in cycles
 *          per microsecond.
 * @param   None
 * @retval  None
 */
static void SensI2C_InitCycleCounter(void)
{
    RCC_ClocksTypeDef clocks;
    uint32_t          first;
    uint32_t          second;
    uint32_t          spin;

    SENS_DEMCR      |= SENS_DEMCR_TRCENA;
    SENS_DWT_CYCCNT  = 0u;
    SENS_DWT_CTRL   |= SENS_DWT_CTRL_CYCCNTENA;

    RCC_GetClocksFreqValue(&clocks);
#if CORE_CM7
    SensI2C_CyclesPerUs = (uint32_t)(clocks.M7ClkFreq / 1000000u);
#else
    SensI2C_CyclesPerUs = (uint32_t)(clocks.M4ClkFreq / 1000000u);
#endif

    /* Two reads a few cycles apart always differ on a counter that is
     * actually running. If they do not, the bit timing has no reference and
     * every delay becomes a no-op */
    first = SENS_DWT_CYCCNT;
    for (spin = 0u; spin < 64u; spin++)
    {
        __NOP();
    }
    second = SENS_DWT_CYCCNT;

    if (first == second)
    {
        SensI2C_CyclesPerUs = 0u;
    }
}

/**
 * @name    SensI2C_ReleaseClock
 * @brief   Let SCL rise and wait for it to actually get there.
 * @param   cfg: pins of the bus being driven.
 * @retval  SENS_I2C_OK once SCL reads high, SENS_I2C_ERR_TIMEOUT if a slave
 *          held it down past the budget.
 */
static int32_t SensI2C_ReleaseClock(const SensI2C_PinCfg_t *cfg)
{
    uint32_t guard = SENS_I2C_POLL_TIMEOUT;

    SENS_SCL_HIGH(cfg);

    while ((SENS_SCL_READ(cfg) == 0u) && (guard > 0u))
    {
        guard--;
    }

    return (guard > 0u) ? SENS_I2C_OK : SENS_I2C_ERR_TIMEOUT;
}

/**
 * @name    SensI2C_Start
 * @brief   Issue a start condition, or a repeated start when the bus is busy.
 * @param   cfg: pins of the bus being driven.
 * @retval  SENS_I2C_OK, or SENS_I2C_ERR_BUS_STUCK if SDA is low while the bus
 *          should be idle.
 */
static int32_t SensI2C_Start(const SensI2C_PinCfg_t *cfg)
{
    uint32_t guard;

    SENS_SDA_HIGH(cfg);

    if (SensI2C_ReleaseClock(cfg) != SENS_I2C_OK)
    {
        return SENS_I2C_ERR_TIMEOUT;
    }

    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

    /* Both lines are released. SDA still reading low means something is
     * holding the bus, but the slave that just acknowledged the previous byte
     * needs a moment to let go, so wait before calling it stuck rather than
     * failing on the first sample. */
    guard = SENS_I2C_POLL_TIMEOUT;
    while ((SENS_SDA_READ(cfg) == 0u) && (guard > 0u))
    {
        guard--;
    }

    if (guard == 0u)
    {
        SENS_SCL_LOW(cfg);
        return SENS_I2C_ERR_BUS_STUCK;
    }

    /* SDA falling while SCL is high is what the slaves decode as a start */
    SENS_SDA_LOW(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);
    SENS_SCL_LOW(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

    return SENS_I2C_OK;
}

/**
 * @name    SensI2C_Stop
 * @brief   Issue a stop condition.
 * @param   cfg: pins of the bus being driven.
 * @retval  None
 */
static void SensI2C_Stop(const SensI2C_PinCfg_t *cfg)
{
    SENS_SDA_LOW(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

    (void)SensI2C_ReleaseClock(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

    /* SDA rising while SCL is high is what the slaves decode as a stop */
    SENS_SDA_HIGH(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);
}

/**
 * @name    SensI2C_PutByte
 * @brief   Clock out eight bits, MSB first, then sample the acknowledge.
 * @param   cfg: pins of the bus being driven.
 * @param   Value: byte to send.
 * @retval  SENS_I2C_OK when the slave ACKed, SENS_I2C_ERR_NACK_DATA when it
 *          did not, SENS_I2C_ERR_TIMEOUT if the clock was stretched too long.
 * @note    A slave that is not there NACKs its address byte through this same
 *          path, so the caller cannot tell the two apart from the return code
 *          alone. SensI2C_PutAddr exists to make that distinction.
 */
static int32_t SensI2C_PutByte(const SensI2C_PinCfg_t *cfg, uint8_t Value)
{
    uint8_t i;
    int32_t ret = SENS_I2C_OK;

    for (i = 0u; i < 8u; i++)
    {
        if ((Value & 0x80u) != 0u)
        {
            SENS_SDA_HIGH(cfg);
        }
        else
        {
            SENS_SDA_LOW(cfg);
        }
        Value = (uint8_t)(Value << 1);

        /* Data has to be stable before SCL rises, so set it, settle, then
         * release the clock */
        SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

        if (SensI2C_ReleaseClock(cfg) != SENS_I2C_OK)
        {
            ret = SENS_I2C_ERR_TIMEOUT;
            break;
        }

        SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

        /* SCL is left low and the next iteration's setup delay finishes the
         * low period off. A third delay on top of that is pure margin at
         * 100 kHz and the dominant term at 400 kHz, so it is not there: the
         * low time is that setup delay plus the GPIO and loop overhead, which
         * is what the fast-mode minimum asks for and no more. */
        SENS_SCL_LOW(cfg);
    }

    if (ret != SENS_I2C_OK)
    {
        SENS_SCL_LOW(cfg);
        SENS_SDA_HIGH(cfg);
        return ret;
    }

    /* Ninth clock: release SDA and let the slave drive it. Low is an ACK */
    SENS_SDA_HIGH(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

    if (SensI2C_ReleaseClock(cfg) != SENS_I2C_OK)
    {
        SENS_SCL_LOW(cfg);
        return SENS_I2C_ERR_TIMEOUT;
    }

    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);
    ret = SENS_SDA_READ(cfg) ? SENS_I2C_ERR_NACK_DATA : SENS_I2C_OK;

    SENS_SCL_LOW(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

    return ret;
}

/**
 * @name    SensI2C_PutAddr
 * @brief   Send a slave address byte and report whether it was acknowledged.
 * @param   cfg: pins of the bus being driven.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @param   Read: non-zero to address the slave for reading.
 * @retval  SENS_I2C_OK on ACK, SENS_I2C_ERR_NACK_ADDR when no slave answered,
 *          or another negative SensI2C_Status_t for a bus fault.
 * @note    The address byte goes out through the same primitive as a data byte,
 *          so a missing slave arrives as a data NACK. This is the one place
 *          that gets turned back into SensI2C_ERR_NACK_ADDR, which is what
 *          lets every caller above treat "not there" and "refused the byte"
 *          as the different faults they are.
 */
static int32_t SensI2C_PutAddr(const SensI2C_PinCfg_t *cfg, uint8_t DevAddr7, uint8_t Read)
{
    uint8_t Addr = (Read != 0u) ? SENS_I2C_ADDR_R(DevAddr7) : SENS_I2C_ADDR_W(DevAddr7);
    int32_t ret  = SensI2C_PutByte(cfg, Addr);

    return (ret == SENS_I2C_ERR_NACK_DATA) ? SENS_I2C_ERR_NACK_ADDR : ret;
}

/**
 * @name    SensI2C_StartWrite
 * @brief   Open a transaction, address the slave for writing and hand it a
 *          register address.
 * @param   cfg: pins of the bus being driven.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @param   Reg: register address.
 * @retval  SENS_I2C_OK once the register address was acknowledged, a negative
 *          SensI2C_Status_t otherwise.
 * @note    Both the write and the read transactions open with exactly this
 *          sequence, so it lives here once rather than in each of them.
 */
static int32_t SensI2C_StartWrite(const SensI2C_PinCfg_t *cfg, uint8_t DevAddr7, uint8_t Reg)
{
    int32_t ret = SensI2C_Start(cfg);

    if (ret == SENS_I2C_OK)
    {
        ret = SensI2C_PutAddr(cfg, DevAddr7, 0u);
    }

    if (ret == SENS_I2C_OK)
    {
        ret = SensI2C_PutByte(cfg, Reg);
    }

    return ret;
}

/**
 * @name    SensI2C_GetByte
 * @brief   Clock in eight bits, MSB first, then send the acknowledge.
 * @param   cfg: pins of the bus being driven.
 * @param   Ack: non-zero to ACK the byte, zero to NACK it.
 * @param   pValue: destination for the byte, must not be NULL.
 * @retval  SENS_I2C_OK, or SENS_I2C_ERR_TIMEOUT if the clock was stretched
 *          too long.
 */
static int32_t SensI2C_GetByte(const SensI2C_PinCfg_t *cfg, uint8_t Ack, uint8_t *pValue)
{
    uint8_t value = 0u;
    uint8_t i;

    /* Hand the line to the slave for the whole byte */
    SENS_SDA_HIGH(cfg);

    for (i = 0u; i < 8u; i++)
    {
        value = (uint8_t)(value << 1);

        SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

        if (SensI2C_ReleaseClock(cfg) != SENS_I2C_OK)
        {
            SENS_SCL_LOW(cfg);
            SENS_SDA_HIGH(cfg);
            return SENS_I2C_ERR_TIMEOUT;
        }

        /* Sample while SCL is high: that is the only window in which the
         * slave guarantees the bit is valid */
        SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);
        if (SENS_SDA_READ(cfg))
        {
            value |= 0x01u;
        }

        /* SCL is left low here for the same reason as in SensI2C_PutByte: the
         * acknowledge sequence's own setup delay finishes the low period */
        SENS_SCL_LOW(cfg);
    }

    /* Ninth clock: the master owns the acknowledge */
    if (Ack != 0u)
    {
        SENS_SDA_LOW(cfg);
    }
    else
    {
        SENS_SDA_HIGH(cfg);
    }

    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);
    (void)SensI2C_ReleaseClock(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

    SENS_SCL_LOW(cfg);
    SENS_SDA_HIGH(cfg);
    SensI2C_DelayUs(SENS_I2C_HALF_PERIOD_US);

    *pValue = value;
    return SENS_I2C_OK;
}
