/**
 * @file    App_Sensor_Handle.c
 * @brief   The two I2C sensors: device protocol, unit conversion and sampling.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.3.0
 *
 * @note    See App_Sensor_Handle.h for the register map, the units and what
 *          each public function promises. What is here is the transaction
 *          sequence per device and the sampling policy.
 *
 *          Magnetometer calibration is no longer here: it moved out unchanged
 *          into App_SensorCalib_Handle.c, which this file calls for the key,
 *          the window and the console's calibration line.
 *
 *          Nothing below this file knows a register address or a scale factor:
 *          the readings leave here as milli-g, milli-degrees per second and
 *          milli-gauss, and the console line is only one consumer of them.
 */

#include "App_Sensor_Handle.h"
#include "App_SensorCalib_Handle.h"

/* How many times the magnetometer probe is repeated before the part is called
 * missing, and the gap left between attempts. One attempt used to be all it
 * got, while the radio's own SDIO init retries five times; the part needs time
 * after its supply comes up, and the datasheet carries both a "time to operate
 * device after VDD valid" spec and a VDD/VIO sequence requirement. */
#define SENS_MAG_PROBE_TRIES            (3u)
#define SENS_MAG_PROBE_GAP_MS           (100u)
#include "log.h"
#include "log_cfg.h"
#include "Mcal_TimerTask.h"
#include "main.h"                   /* provides extern volatile uint32_t TimeBase */

/* The scan is capped well above the one device expected per bus: the point is
 * to notice a stray answer as much as a missing one */
#define SENS_SCAN_MAX                   (8u)

/* One sample tick is MCAL_SAMPLE_DIV base ticks and the base rate divides a
 * second exactly, so this period is exact and the microsecond clock below is a
 * count rather than a measurement. */
#define SENS_SAMPLE_PERIOD_US           ((1000000u / MCAL_SAMPLE_TICK_HZ) * MCAL_SAMPLE_DIV)

static App_Sensor_Sample_t Sens_Sample;
static uint8_t  Sens_ImuAddr = 0u;
static uint8_t  Sens_MagReady = 0u;
static uint8_t  Sens_Fresh = 0u;
static uint32_t Sens_LastLogTick = 0u;
static uint32_t Sens_BackoffTick = 0u;

/* The microsecond clock the samples are stamped with, counted from the tick.
 * Nothing reads a timer back: the tick period is exact, so counting it is exact
 * too, and a free-running microsecond counter would wrap inside an hour. */
static uint32_t Sens_TimeUs = 0u;

/* Divides the sample tick down to the rate the field is read at */
static uint8_t  Sens_MagDivCount = 0u;

/* Which chains delivered at least one sample since the last console line. The
 * line reports each live chain's latest values, and this is what says which of
 * them are live: a sample's own Flags describe one tick, and at the sample rate
 * the magnetometer lands in only a quarter of them, so a line gated on those
 * would drop the field three windows out of four. */
static uint8_t  Sens_Seen = 0u;

/* Failed transfers, counted rather than printed: there is no console in the
 * tick. The console reports them from the main loop whenever the total moves,
 * which is what keeps a sensor that has come loose from costing a UART write
 * per tick. */
static uint32_t Sens_ImuErrCount = 0u;
static uint32_t Sens_MagErrCount = 0u;
static uint32_t Sens_ErrLogged = 0u;

#if (SENS_PLOT_FIREWATER != 0)
/* The plot's own gate, which runs on its own period and is untouched by the
 * console's */
static uint32_t Sens_LastPlotTick = 0u;
#endif

/* Calibration state used to live here; it is in App_SensorCalib_Handle.c now,
 * behind the calls this file makes into that module. */

static int32_t Sens_ScanBus(SensI2C_Bus_t Bus, const char *pName,
                            uint8_t *pFound, uint8_t MaxFound);
static int Sens_ReadIdentity(SensI2C_Bus_t Bus, uint8_t DevAddr7, const char *pName,
                             uint8_t Reg, uint8_t Expected);
static int Sens_ImuWriteVerify(uint8_t DevAddr7, uint8_t Reg, uint8_t Value, const char *pWhat);
static int Sens_ImuConfig(uint8_t DevAddr7);
static int Sens_ImuRead(App_Sensor_Sample_t *pSample);
static int Sens_MagConfig(void);
static int Sens_MagWriteVerify(uint8_t Reg, uint8_t Value, const char *pWhat);
static int Sens_MagRead(App_Sensor_Sample_t *pSample);
#if (SENS_PLOT_FIREWATER != 0)
static void Sens_PlotFrame(const App_Sensor_Sample_t *pSample);
#endif

/**
 * @name    Sens_ScanBus
 * @brief   Bring up one bus, scan it and print every address that answered.
 * @param   Bus: the bus to bring up.
 * @param   pName: part expected on this bus, for the log only.
 * @param   pFound: destination array for the addresses that ACKed.
 * @param   MaxFound: capacity of pFound.
 * @retval  Number of addresses found, or a negative SensI2C_Status_t.
 */
static int32_t Sens_ScanBus(SensI2C_Bus_t Bus, const char *pName,
                            uint8_t *pFound, uint8_t MaxFound)
{
    int32_t Count;
    uint32_t ScanTick;
    uint32_t ScanMs;
    uint8_t SclHigh = 0u;
    uint8_t SdaHigh = 0u;
    uint8_t i;

    if (Mcal_SensI2C_Init(Bus) != SENS_I2C_OK)
    {
        log_info("[SENS] bus %d (%s) init failed, SDA is held low at idle\r\n",
                 (int)Bus, pName);
        return SENS_I2C_ERR_BUS_STUCK;
    }

    log_info("[SENS] bus %d (%s) up, %d Hz\r\n", (int)Bus, pName, (int)SENS_I2C_BUS_HZ);

    if (Mcal_SensI2C_GetLineLevels(Bus, &SclHigh, &SdaHigh) == SENS_I2C_OK)
    {
        log_info("[SENS] bus %d idle: SCL=%s SDA=%s\r\n", (int)Bus,
                 (SclHigh != 0u) ? "high" : "LOW",
                 (SdaHigh != 0u) ? "high" : "LOW");
    }

    /* Timed around the scan alone. There is no console output inside this
     * window, which is what makes this a bus figure rather than a boot
     * figure: fputc() blocks, so anything with a print in it measures the
     * console. */
    ScanTick = TimeBase;
    Count    = Mcal_SensI2C_Scan(Bus, pFound, MaxFound);
    ScanMs   = (uint32_t)(TimeBase - ScanTick);

    if (Count < 0)
    {
        log_info("[SENS] bus %d (%s) scan aborted, ret = %d\r\n", (int)Bus, pName, (int)Count);
        return Count;
    }

    log_info("[SENS] bus %d scan found %d in %d ms:", (int)Bus, (int)Count, (int)ScanMs);
    for (i = 0u; (i < (uint8_t)Count) && (i < MaxFound); i++)
    {
        log_info(" 0x%02X", pFound[i]);
    }
    log_info("\r\n");

    return Count;
}

/**
 * @name    Sens_ReadIdentity
 * @brief   Read one device's identity register and print it.
 * @param   Bus: bus the device sits on.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @param   pName: short part name, for the log only.
 * @param   Reg: identity register address.
 * @param   Expected: documented value, used only to flag a mismatch.
 * @retval  SENS_OK when the register was read, SENS_ERR_ID when it was not.
 * @note    A mismatch is reported but not treated as an error: the value is
 *          the point of the step, and the magnetometer's is genuinely in
 *          doubt. Only a failed transfer is a failure here.
 */
static int Sens_ReadIdentity(SensI2C_Bus_t Bus, uint8_t DevAddr7, const char *pName,
                             uint8_t Reg, uint8_t Expected)
{
    uint8_t Value = 0u;
    int32_t ret;

    ret = Mcal_SensI2C_ReadReg(Bus, DevAddr7, Reg, &Value);
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] %s id reg 0x%02X read failed, ret = %d\r\n", pName, Reg, (int)ret);
        return SENS_ERR_ID;
    }

    log_info("[SENS] %s id (0x%02X) = 0x%02X%s\r\n", pName, Reg, Value,
             (Value == Expected) ? "" : "   <- unexpected");

    return SENS_OK;
}

/**
 * @name    Sens_ImuWriteVerify
 * @brief   Write one IMU register and read it straight back, so a write that
 *          did not take is reported where it happened rather than surfacing
 *          later as a reading that makes no sense.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @param   Reg: register address.
 * @param   Value: byte to write.
 * @param   pWhat: register name for the log only.
 * @retval  SENS_OK when the register reads back as written, SENS_ERR_IMU_CONFIG
 *          when a transfer failed or the value did not match.
 * @note    A write that is not read back proves nothing. Every configuration
 *          register this file touches goes through here for that reason, and
 *          the read-back is what tells a write fault apart from a read fault
 *          when the measurements misbehave later.
 */
static int Sens_ImuWriteVerify(uint8_t DevAddr7, uint8_t Reg, uint8_t Value, const char *pWhat)
{
    uint8_t Back = 0u;
    int32_t ret;

    ret = Mcal_SensI2C_WriteReg(SENS_IMU_BUS, DevAddr7, Reg, Value);
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] IMU %s (0x%02X) write failed, ret = %d\r\n", pWhat, Reg, (int)ret);
        return SENS_ERR_IMU_CONFIG;
    }

    ret = Mcal_SensI2C_ReadReg(SENS_IMU_BUS, DevAddr7, Reg, &Back);
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] IMU %s (0x%02X) read-back failed, ret = %d\r\n", pWhat, Reg, (int)ret);
        return SENS_ERR_IMU_CONFIG;
    }

    log_info("[SENS] IMU %s (0x%02X) = 0x%02X%s\r\n", pWhat, Reg, Back,
             (Back == Value) ? "" : "   <- write did not take");

    return (Back == Value) ? SENS_OK : SENS_ERR_IMU_CONFIG;
}

/**
 * @name    Sens_ImuConfig
 * @brief   Configure both measurement chains of the IMU: rate on the
 *          accelerometer, range and rate on the gyroscope.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @retval  SENS_OK when every register read back as written, SENS_ERR_ID when
 *          the control-register read failed, SENS_ERR_IMU_CONFIG otherwise.
 * @note    CTRL3 is read first because burst reads depend on its IF_INC bit,
 *          and it is only reported, not corrected: 0x44 is the documented
 *          reset value, so a different one means something reset the part or
 *          the address is wrong, and either is worth knowing before the
 *          measurements are blamed for it.
 * @note    The gyroscope's range is written before its rate because the range
 *          is what the readings get scaled by.
 */
static int Sens_ImuConfig(uint8_t DevAddr7)
{
    uint8_t Ctrl3 = 0u;
    int32_t ret;

    ret = Mcal_SensI2C_ReadReg(SENS_IMU_BUS, DevAddr7, SENS_LSM6DSV16X_REG_CTRL3, &Ctrl3);		//0x44
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] IMU CTRL3 read failed, ret = %d\r\n", (int)ret);
        return SENS_ERR_ID;
    }

    log_info("[SENS] IMU CTRL3 (0x%02X) = 0x%02X%s\r\n", SENS_LSM6DSV16X_REG_CTRL3, Ctrl3,
             (Ctrl3 == SENS_LSM6DSV16X_CTRL3_DEFAULT) ? "" : "   <- unexpected");

    if (Sens_ImuWriteVerify(DevAddr7, SENS_LSM6DSV16X_REG_CTRL1,
                            SENS_LSM6DSV16X_CTRL1_ODR_120HZ, "CTRL1") != SENS_OK)		//设置加速度计的输出数据率 ODR 为 120 Hz
    {
        return SENS_ERR_IMU_CONFIG;
    }

    if (Sens_ImuWriteVerify(DevAddr7, SENS_LSM6DSV16X_REG_CTRL6,
                            SENS_LSM6DSV16X_CTRL6_FS_500DPS, "CTRL6") != SENS_OK)		//设置陀螺仪的量程 FS 为 ±500 dps
    {
        return SENS_ERR_IMU_CONFIG;
    }

    if (Sens_ImuWriteVerify(DevAddr7, SENS_LSM6DSV16X_REG_CTRL2,
                            SENS_LSM6DSV16X_CTRL2_ODR_120HZ, "CTRL2") != SENS_OK)		//设置陀螺仪的输出数据率 ODR 为 120 Hz。
    {
        return SENS_ERR_IMU_CONFIG;
    }

    return SENS_OK;
}

/**
 * @name    Sens_ImuRead
 * @brief   Read whichever of the IMU's two chains has a fresh sample and write
 *          it into the sample buffer in engineering units.
 * @param   pSample: sample buffer to fill in.
 * @retval  A mask of APP_SENSOR_HAVE_ACCEL and APP_SENSOR_HAVE_GYRO for the
 *          chains that produced data, 0 when neither had a sample ready, or a
 *          negative SensI2C_Status_t when a transfer failed.
 * @note    STATUS_REG carries one ready flag per chain, so it is read once for
 *          both rather than once each. It stays a transaction of its own
 *          because it is what decides whether the burst below is worth sending
 *          at all: a tick with neither flag set returns without touching the
 *          bus.
 * @note    The two output chains are adjacent in the register map, so one
 *          twelve-byte burst from OUTX_L_G returns both and spends one
 *          transaction's worth of START, address and STOP instead of two,
 *          which is about a fifth of what this read used to cost on the wire.
 *          The gyroscope is first because 0x22 precedes 0x28, and the offsets
 *          below are that order and nothing else.
 * @note    Reading a chain's output registers is what clears its own flag, and
 *          the two do not disturb each other, so a burst sent on one chain's
 *          account cannot make the other's next ready flag disappear.
 */
static int Sens_ImuRead(App_Sensor_Sample_t *pSample)
{
    uint8_t Status = 0u;
    uint8_t Raw[SENS_IMU_BURST_BYTES];
    int32_t ret;
    int Fresh = 0;

    ret = Mcal_SensI2C_ReadReg(SENS_IMU_BUS, Sens_ImuAddr, SENS_LSM6DSV16X_REG_STATUS, &Status);		//读状态寄存器
    if (ret != SENS_I2C_OK)
    {
        return (int)ret;
    }

    if (((Status & SENS_LSM6DSV16X_STATUS_XLDA) == 0u) &&
        ((Status & SENS_LSM6DSV16X_STATUS_GDA) == 0u))																							//无新数据
    {
        return 0;
    }

    ret = Mcal_SensI2C_ReadRegs(SENS_IMU_BUS, Sens_ImuAddr, SENS_LSM6DSV16X_REG_OUTX_L_G,						//读输出寄存器12字节
                                Raw, SENS_IMU_BURST_BYTES);
    if (ret != SENS_I2C_OK)
    {
        return (int)ret;
    }

    if ((Status & SENS_LSM6DSV16X_STATUS_GDA) != 0u)																								//陀螺仪6字节数据
    {
        /* Little-endian pairs, half a word per axis; gyroscope is the first half */
        pSample->GyroMdpsX = SENS_IMU_GYRO_MDPS((uint16_t)Raw[0] | ((uint16_t)Raw[1] << 8));
        pSample->GyroMdpsY = SENS_IMU_GYRO_MDPS((uint16_t)Raw[2] | ((uint16_t)Raw[3] << 8));
        pSample->GyroMdpsZ = SENS_IMU_GYRO_MDPS((uint16_t)Raw[4] | ((uint16_t)Raw[5] << 8));
        pSample->Flags    |= APP_SENSOR_HAVE_GYRO;
        Fresh             |= APP_SENSOR_HAVE_GYRO;
    }

    if ((Status & SENS_LSM6DSV16X_STATUS_XLDA) != 0u)																								//加速度计6字节数据
    {
        /* Same layout, second half: 0x28 follows 0x27 */
        pSample->AccelMgX = SENS_IMU_ACCEL_MG((uint16_t)Raw[6] | ((uint16_t)Raw[7] << 8));
        pSample->AccelMgY = SENS_IMU_ACCEL_MG((uint16_t)Raw[8] | ((uint16_t)Raw[9] << 8));
        pSample->AccelMgZ = SENS_IMU_ACCEL_MG((uint16_t)Raw[10] | ((uint16_t)Raw[11] << 8));
        pSample->Flags   |= APP_SENSOR_HAVE_ACCEL;
        Fresh            |= APP_SENSOR_HAVE_ACCEL;
    }

    return Fresh;
}

/**
 * @name    Sens_MagWriteVerify
 * @brief   Write one magnetometer control register and read it back.
 * @param   Reg: register address.
 * @param   Value: value to write.
 * @param   pWhat: short register name, for the console.
 * @retval  SENS_OK when the read-back matches, SENS_ERR_MAG_CONFIG when a
 *          transfer failed or the value did not stick.
 * @note    Silent on success: Sens_MagConfig() reports all four registers on
 *          one line once they have landed.
 */
static int Sens_MagWriteVerify(uint8_t Reg, uint8_t Value, const char *pWhat)
{
    uint8_t Back = 0u;
    int32_t ret;

    ret = Mcal_SensI2C_WriteReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7, Reg, Value);											//写寄存器
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] MAG %s (0x%02X) write failed, ret = %d\r\n", pWhat, Reg, (int)ret);
        return SENS_ERR_MAG_CONFIG;
    }

    ret = Mcal_SensI2C_ReadReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7, Reg, &Back);												//读寄存器
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] MAG %s (0x%02X) read-back failed, ret = %d\r\n", pWhat, Reg, (int)ret);
        return SENS_ERR_MAG_CONFIG;
    }

    if (Back != Value)
    {
        log_info("[SENS] MAG %s (0x%02X) = 0x%02X, wanted 0x%02X\r\n", pWhat, Reg, Back, Value);
        return SENS_ERR_MAG_CONFIG;
    }

    return SENS_OK;
}

/**
 * @name    Sens_MagConfig
 * @brief   Put the magnetometer into continuous mode at 100 Hz and prove it
 *          took.
 * @param   None
 * @retval  SENS_OK when the part is measurably running, SENS_ERR_MAG_CONFIG
 *          otherwise.
 * @note    The write order is the datasheet's and it matters: a non-zero ODR
 *          first, then Cmm_freq_en so the part can derive its counter target,
 *          then Cmm_en to start it. Any other order leaves the part idle,
 *          which on this board looks exactly like a missing device.
 * @note    The bandwidth goes before the rate because the rate is bounded by
 *          it. At the reset value of 00 the decimation is 6.6 ms long and
 *          continuous mode tops out at 75 Hz, so 100 Hz needs the 10 setting:
 *          2 ms per measurement against a 255 Hz ceiling, which finishes well
 *          inside the 10 ms tick that reads it.
 * @note    Control register 0 is used as a handshake rather than as a
 *          read-back: the part clears Cmm_freq_en itself once the counter
 *          target has been worked out, and Cmm_en must not be written before
 *          that. Only Auto_SR_en is required to persist.
 * @note    The last step is the one that means anything, because three control
 *          registers can read back as written while nothing is being measured.
 *          The mode is accepted only once a measurement has actually landed,
 *          and the flag is cleared first so that the wait can only be
 *          satisfied by a measurement this code caused. Meas_m_done is left
 *          set on the way out for the poll to collect, since reading the data
 *          registers is what clears it.
 */
static int Sens_MagConfig(void)
{
    uint8_t Odr = 0u;
    uint8_t Ctrl1 = 0u;
    uint8_t Ctrl0 = 0u;
    uint8_t Ctrl2 = 0u;
    uint8_t Status = 0u;
    uint8_t Discard[SENS_MAG_DATA_BYTES];
    uint32_t Start;
    int32_t ret;

    if (Sens_MagWriteVerify(SENS_MMC5617WA_REG_CTRL1, SENS_MMC5617WA_CTRL1_BW10,
                            "CTRL1 BW") != SENS_OK)																				//配置带宽
    {
        return SENS_ERR_MAG_CONFIG;
    }
    if (Sens_MagWriteVerify(SENS_MMC5617WA_REG_ODR, SENS_MMC5617WA_ODR_HZ,
                            "ODR") != SENS_OK)																						//ODR设置100HZ
    {
        return SENS_ERR_MAG_CONFIG;
    }

    /* Auto_SR_en and Cmm_freq_en go in together, so the mode starts with the
     * periodic set already enabled. */
    ret = Mcal_SensI2C_WriteReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                                SENS_MMC5617WA_REG_CTRL0,
                                (uint8_t)(SENS_MMC5617WA_CTRL0_AUTO_SR |
                                          SENS_MMC5617WA_CTRL0_CMM_FREQ));								//配置自动模式和连续测量频率计算寄存器
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] MAG CTRL0 write failed, ret = %d\r\n", (int)ret);
        return SENS_ERR_MAG_CONFIG;
    }

    /* Cmm_en must not be written until the part has worked out the counter
     * target, and the way to know it has is that the part clears the command
     * bit itself. So this is a handshake rather than a read-back: wait for
     * Cmm_freq_en to fall, and only then start the mode. The same read shows
     * what persisted in the byte, which is where Auto_SR_en is checked. The
     * timeout is one ODR period, orders of magnitude more than the calculation
     * needs; expiring is not fatal on its own, because the measurement wait
     * below is what actually decides whether the mode started. */
    Start = TimeBase;
    for (;;)
    {
        ret = Mcal_SensI2C_ReadReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                                   SENS_MMC5617WA_REG_CTRL0, &Ctrl0);
        if (ret != SENS_I2C_OK)
        {
            log_info("[SENS] MAG CTRL0 read-back failed, ret = %d\r\n", (int)ret);
            return SENS_ERR_MAG_CONFIG;
        }
        if ((Ctrl0 & SENS_MMC5617WA_CTRL0_AUTO_SR) == 0u)
        {
            log_info("[SENS] MAG CTRL0 (0x%02X) = 0x%02X, Auto_SR_en did not stick\r\n",
                     SENS_MMC5617WA_REG_CTRL0, (unsigned)Ctrl0);
            return SENS_ERR_MAG_CONFIG;
        }
        if (((Ctrl0 & SENS_MMC5617WA_CTRL0_CMM_FREQ) == 0u) ||	
            ((TimeBase - Start) >= SENS_MAG_STARTUP_MS))																						//等待磁力计计算配置
        {
            break;
        }
    }

    /* Reading the data registers clears Meas_m_done, so this discards whatever
     * an earlier run left behind before the mode is started. */
    ret = Mcal_SensI2C_ReadRegs(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                                SENS_MMC5617WA_REG_XOUT0, Discard,
                                SENS_MAG_DATA_BYTES);																								//丢弃旧数据
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] MAG data clear failed, ret = %d\r\n", (int)ret);
        return SENS_ERR_MAG_CONFIG;
    }

    if (Sens_MagWriteVerify(SENS_MMC5617WA_REG_CTRL2, SENS_MMC5617WA_CTRL2_CMM_EN,									//写CTRL2，启动连续测量模式
                            "CTRL2 Cmm_en") != SENS_OK)
    {
        return SENS_ERR_MAG_CONFIG;
    }

    Start = TimeBase;
    for (;;)
    {
        ret = Mcal_SensI2C_ReadReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                                   SENS_MMC5617WA_REG_STATUS, &Status);
        if (ret != SENS_I2C_OK)
        {
            log_info("[SENS] MAG status read failed, ret = %d\r\n", (int)ret);
            return SENS_ERR_MAG_CONFIG;
        }
        if ((Status & SENS_MMC5617WA_STATUS_MEAS_M) != 0u)
        {
            break;
        }
        if ((TimeBase - Start) >= SENS_MAG_STARTUP_MS)																							//等待测量完成
        {
            log_info("[SENS] MAG no measurement within %d ms, status = 0x%02X\r\n",
                     (int)SENS_MAG_STARTUP_MS, (unsigned)Status);
            return SENS_ERR_MAG_CONFIG;
        }
    }

    /* Reported from a second pass over the registers, so the line says what
     * the part holds rather than what it was asked for. */
    ret = Mcal_SensI2C_ReadReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                               SENS_MMC5617WA_REG_ODR, &Odr);																				//回读ODR寄存器配置
    if (ret == SENS_I2C_OK)
    {
        ret = Mcal_SensI2C_ReadReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                                   SENS_MMC5617WA_REG_CTRL1, &Ctrl1);
    }
    if (ret == SENS_I2C_OK)
    {
        ret = Mcal_SensI2C_ReadReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                                   SENS_MMC5617WA_REG_CTRL0, &Ctrl0);
    }
    if (ret == SENS_I2C_OK)
    {
        ret = Mcal_SensI2C_ReadReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                                   SENS_MMC5617WA_REG_CTRL2, &Ctrl2);
    }
    if (ret != SENS_I2C_OK)
    {
        log_info("[SENS] MAG read-back failed, ret = %d\r\n", (int)ret);
        return SENS_ERR_MAG_CONFIG;
    }

    log_info("[SENS] MAG ODR=%dHz BW1:BW0=%d Auto_SR=%d Cmm_en=%d, measuring after"
             " %d ms -- continuous, 10 ms tick\r\n",
             (int)Odr, (int)(Ctrl1 & 0x03u),
             (int)((Ctrl0 & SENS_MMC5617WA_CTRL0_AUTO_SR) != 0u),
             (int)((Ctrl2 & SENS_MMC5617WA_CTRL2_CMM_EN) != 0u),
             (int)(TimeBase - Start));

    return SENS_OK;
}

/**
 * @name    Sens_MagRead
 * @brief   Read the three magnetic axes into the sample buffer in milli-gauss,
 *          if a measurement has finished.
 * @param   pSample: sample buffer to fill in.
 * @retval  APP_SENSOR_HAVE_MAG when a fresh sample was decoded, 0 when the
 *          measurement was not finished yet, or a negative SensI2C_Status_t
 *          when a transfer failed.
 * @note    The eight bytes are read in one transfer, partly so the axes come
 *          from the same measurement and partly because reading any magnetic
 *          data register is what clears Meas_m_done -- reading them one at a
 *          time would take three passes over the same flag.
 */
static int Sens_MagRead(App_Sensor_Sample_t *pSample)
{
    uint8_t Status = 0u;
    uint8_t Raw[SENS_MAG_DATA_BYTES];
    uint32_t X;
    uint32_t Y;
    uint32_t Z;
    int32_t ret;

    ret = Mcal_SensI2C_ReadReg(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                               SENS_MMC5617WA_REG_STATUS, &Status);															//读状态寄存器
    if (ret != SENS_I2C_OK)
    {
        return (int)ret;
    }

    if ((Status & SENS_MMC5617WA_STATUS_MEAS_M) == 0u)																					//无新数据
    {
        return 0;
    }

    ret = Mcal_SensI2C_ReadRegs(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7,
                                SENS_MMC5617WA_REG_XOUT0, Raw, SENS_MAG_DATA_BYTES);						//读取8字节
    if (ret != SENS_I2C_OK)	
    {
        return (int)ret;
    }

    X = ((uint32_t)Raw[0] << 12) | ((uint32_t)Raw[1] << 4) | ((uint32_t)Raw[6] >> 4);						
    Y = ((uint32_t)Raw[2] << 12) | ((uint32_t)Raw[3] << 4) | ((uint32_t)Raw[6] & 0x0Fu);				//拼接三轴数据
    Z = ((uint32_t)Raw[4] << 12) | ((uint32_t)Raw[5] << 4) | ((uint32_t)Raw[7] >> 4);

    pSample->MagMgaussX = SENS_MMC5617WA_MG(X);
    pSample->MagMgaussY = SENS_MMC5617WA_MG(Y);																									//更新数据
    pSample->MagMgaussZ = SENS_MMC5617WA_MG(Z);
    pSample->Flags     |= APP_SENSOR_HAVE_MAG;

    return APP_SENSOR_HAVE_MAG;																																	//返回标志位
}


/*求算术平方根（向下取整）*/
uint32_t Sens_Isqrt(uint32_t Value)
{
    uint32_t Root = 0u;
    uint32_t Bit = 1u << 30;

    while (Bit > Value)
    {
        Bit >>= 2;
    }

    while (Bit != 0u)
    {
        if (Value >= (Root + Bit))
        {
            Value -= (Root + Bit);
            Root = (Root >> 1) + Bit;
        }
        else
        {
            Root >>= 1;
        }
        Bit >>= 2;
    }

    return Root;
}

int App_Sensor_Init(void)
{
    uint8_t Found[SENS_SCAN_MAX];
    uint8_t ImuAddr;
    uint8_t Mag_Missing = 0u;
    int32_t ret;
    int32_t ImuLowRet;
    int32_t ImuHighRet;
    uint32_t StartTick = TimeBase;

    /* The calibration key is brought up and reported first, ahead of any sensor
     * work; the pad, its idle-level check and why both come first are in
     * App_SensorCalib_Handle.c. */
    App_SensorCalib_Init();																															//初始化标定按键

    /* Bus 0 carries the magnetometer and nothing else */
    if (Sens_ScanBus(SENS_MAG_BUS, "MMC5617WA", Found, SENS_SCAN_MAX) < 0)
    {
        return SENS_ERR_BUS;
    }

    /* The part is given more than one chance before it is called missing. */
    for (uint8_t Try = 0u; Try < SENS_MAG_PROBE_TRIES; Try++)
    {
        ret = Mcal_SensI2C_Probe(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7);
        if (ret == SENS_I2C_OK)
        {
            break;
        }
        if ((Try + 1u) < SENS_MAG_PROBE_TRIES)
        {
            uint32_t WaitStart = TimeBase;

            log_info("[SENS] MMC5617WA  @ 0x%02X : no answer, retry %d of %d\r\n",
                     SENS_MMC5617WA_ADDR7, (int)(Try + 2u), (int)SENS_MAG_PROBE_TRIES);
            while ((TimeBase - WaitStart) < SENS_MAG_PROBE_GAP_MS)
            {
            }
        }
    }

    log_info("[SENS] MMC5617WA  @ 0x%02X : %s\r\n", SENS_MMC5617WA_ADDR7,
             (ret == SENS_I2C_OK) ? "ACK" : "no answer");

    if (ret != SENS_I2C_OK)
    {
        /* Kept, not fatal: the two parts are on separate buses, and a missing
         * magnetometer is no reason to leave the six axis unconfigured. The
         * error is returned at the end of this function instead, once the six
         * axis has had its turn. */
        Mag_Missing = 1u;
    }
		
    if (Sens_ScanBus(SENS_IMU_BUS, "LSM6DSV16X", Found, SENS_SCAN_MAX) < 0)
    {
        return SENS_ERR_SCAN;
    }

    ImuLowRet  = Mcal_SensI2C_Probe(SENS_IMU_BUS, SENS_LSM6DSV16X_ADDR7_SA0_LOW);
    ImuHighRet = Mcal_SensI2C_Probe(SENS_IMU_BUS, SENS_LSM6DSV16X_ADDR7_SA0_HIGH);

    log_info("[SENS] LSM6DSV16X @ 0x%02X : %s\r\n", SENS_LSM6DSV16X_ADDR7_SA0_LOW,
             (ImuLowRet == SENS_I2C_OK) ? "ACK" : "no answer");
    log_info("[SENS] LSM6DSV16X @ 0x%02X : %s\r\n", SENS_LSM6DSV16X_ADDR7_SA0_HIGH,
             (ImuHighRet == SENS_I2C_OK) ? "ACK" : "no answer");

    if ((ImuLowRet != SENS_I2C_OK) && (ImuHighRet != SENS_I2C_OK))
    {
        return SENS_ERR_IMU_MISSING;
    }

    ImuAddr = (ImuLowRet == SENS_I2C_OK) ? (uint8_t)SENS_LSM6DSV16X_ADDR7_SA0_LOW
                                         : (uint8_t)SENS_LSM6DSV16X_ADDR7_SA0_HIGH;

    /* The magnetometer's own two steps, skipped when it never answered. The
     * body is left at its original indentation on purpose: re-indenting it
     * would mean rewriting a line that carries a Chinese end-of-line comment,
     * and this patch does not touch any non-ASCII byte. */
    if (Mag_Missing == 0u)
    {
    if (Sens_ReadIdentity(SENS_MAG_BUS, SENS_MMC5617WA_ADDR7, "MAG",
                          SENS_MMC5617WA_REG_PRODUCT_ID,
                          SENS_MMC5617WA_PRODUCT_ID) != SENS_OK)
    {
        return SENS_ERR_ID;
    }

    if (Sens_MagConfig() != SENS_OK)																											//配置磁力计
    {
        return SENS_ERR_MAG_CONFIG;
    }
    }

    if (Sens_ReadIdentity(SENS_IMU_BUS, ImuAddr, "IMU",
                          SENS_LSM6DSV16X_REG_WHO_AM_I,
                          SENS_LSM6DSV16X_WHO_AM_I) != SENS_OK)
    {
        return SENS_ERR_ID;
    }

    /* Only now that the part is known to answer is anything written to it */
    if (Sens_ImuConfig(ImuAddr) != SENS_OK)																								//配置IMU
    {
        return SENS_ERR_IMU_CONFIG;
    }

    Sens_ImuAddr     = ImuAddr;
    if (Mag_Missing == 0u)
    {
        Sens_MagReady = 1u;
    }
    Sens_LastLogTick  = TimeBase;

    /* The magnetometer is up, so the delayed automatic window -- if the build
     * has one -- can be armed. The condition is inside that module. */
    App_SensorCalib_Ready();																															//标定就绪

    log_info("[SENS] init total %d ms\r\n", (int)(TimeBase - StartTick));

    if (Mag_Missing != 0u)
    {
        return SENS_ERR_MAG_MISSING;
    }

    return SENS_OK;
}

int App_Sensor_UpdateData_Tick(void)
{
    uint32_t Now = TimeBase;
    int ImuRet;
    int MagRet;

    /* Counted, not measured: the tick period is an exact division of the clock,
     * so this clock is exact to that period and never depends on the main loop
     * reaching a slot. */
    Sens_TimeUs += SENS_SAMPLE_PERIOD_US;

    if ((Sens_ImuAddr == 0u) && (Sens_MagReady == 0u))
    {
        return 0;                   /* Neither part came up at init */
    }
    if (Sens_BackoffTick != 0u)
    {
        if ((Now - Sens_BackoffTick) < SENS_LOG_PERIOD_MS)
        {
            return 0;
        }
        Sens_BackoffTick = 0u;
    }

    if (Sens_ImuAddr != 0u)
    {
        ImuRet = Sens_ImuRead(&Sens_Sample);																					//读取IMU数据
        if (ImuRet < 0)
        {
            Sens_ImuErrCount++;
            Sens_BackoffTick = Now;
        }
        else
        {
            Sens_Fresh |= (uint8_t)ImuRet;
        }
    }

    /* The field turns far more slowly than the body does, so it is read on a
     * division of the sample rate rather than on every tick */
    if (Sens_MagReady != 0u)																												//读取磁力计数据
    {
        Sens_MagDivCount++;
        if (Sens_MagDivCount >= SENS_MAG_DIV)
        {
            Sens_MagDivCount = 0u;

            MagRet = Sens_MagRead(&Sens_Sample);
            if (MagRet < 0)
            {
                Sens_MagErrCount++;
                Sens_BackoffTick = Now;
            }
            else
            {
                Sens_Fresh |= (uint8_t)MagRet;
                if ((App_SensorCalib_IsActive() != 0u) &&
                    (MagRet == APP_SENSOR_HAVE_MAG))
                {
                    Sens_CalibAccumulate(&Sens_Sample);
                }
            }
        }
    }

    if (Sens_Fresh == 0u)
    {
        return 0;
    }

    /* Replaced rather than OR-ed in: Flags describes the tick that produced
     * both timestamps, which is what lets a consumer tell a fresh reading from
     * the one before it. While it accumulated, nothing read it, and the filter
     * would have taken a magnetometer value minutes old for a current one. */
    Sens_Sample.TickMs = Now;
    Sens_Sample.TickUs = Sens_TimeUs;
    Sens_Sample.Flags  = Sens_Fresh;

    /* Kept separately for the console, which reports over a window rather than
     * over one tick: what it needs is which chains are live, not which of them
     * happened to land in the tick it samples. */
    Sens_Seen |= Sens_Fresh;
    Sens_Fresh = 0u;

    return 1;
}

#if (SENS_PLOT_FIREWATER != 0)
/**
 * @name    Sens_PlotFrame
 * @brief   Print one FireWater frame: the nine channels the plot carries.
 * @param   pSample: the sample those channels are read from.
 * @retval  None
 * @note    One frame per line, channels separated by commas and nothing else --
 *          no spaces, no units, no prefix. Integers rather than floats because
 *          the readings already are integers: that is both the exact value and
 *          the shortest frame, and the frame length is what costs time here.
 * @note    This blocks. The console writes a character at a time, so a frame is
 *          milliseconds of the calling loop, which is why this runs from the
 *          main loop and never from the sampling tick, and why the period is
 *          picked against the baud rate rather than against the sample rate.
 */
static void Sens_PlotFrame(const App_Sensor_Sample_t *pSample)
{
    log_info("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
             (int)pSample->AccelMgX, (int)pSample->AccelMgY, (int)pSample->AccelMgZ,
             (int)pSample->GyroMdpsX, (int)pSample->GyroMdpsY, (int)pSample->GyroMdpsZ,
             (int)pSample->MagMgaussX, (int)pSample->MagMgaussY, (int)pSample->MagMgaussZ);
}
#endif

void App_Sensor_Poll(void)
{
    uint32_t Now = TimeBase;
    uint32_t ImuErr;
    uint32_t MagErr;
    App_Sensor_Sample_t Snapshot;
    uint8_t  Seen;
    uint8_t  LogDue;
    uint8_t  PlotDue = 0u;

    /* The sampling runs on the base tick, so what is left for a slot in the main
     * loop is the two things that must not run in an interrupt: the key, which
     * wants a slow steady cadence to debounce against, and the console, which
     * waits on the transmitter. The key belongs to the calibration module, which
     * polls it ahead of the console gates below so that a press is still seen
     * while a calibration window is closing it. */
    App_SensorCalib_Poll(Now);

    /* Two independent gates: the console's line per period, and the plot frame
     * on its own much faster one. Neither does any work until it is due, and the
     * sample is copied once for whichever of them needs it. */
    LogDue = ((Now - Sens_LastLogTick) >= SENS_LOG_PERIOD_MS) ? 1u : 0u;
#if (SENS_PLOT_FIREWATER != 0)
    PlotDue = ((Now - Sens_LastPlotTick) >= SENS_PLOT_PERIOD_MS) ? 1u : 0u;
#endif
    if ((LogDue == 0u) && (PlotDue == 0u))
    {
        return;
    }
    if (LogDue != 0u)
    {
        Sens_LastLogTick = Now;
    }
#if (SENS_PLOT_FIREWATER != 0)
    if (PlotDue != 0u)
    {
        Sens_LastPlotTick = Now;
    }
#endif

    /* The sample and the failure counts are read under the mask because both are
     * written from the tick: a line built from a live read could mix two samples
     * into one, and a count copied while it is being stepped could lose one.
     * Seen belongs to the console's window, so only the console's gate clears it
     * -- a frame at ten times that rate would otherwise empty it, and the axes
     * line would lose whichever chains landed in between. */
    __disable_irq();
    Snapshot = Sens_Sample;
    Seen     = Sens_Seen;
    if (LogDue != 0u)
    {
        Sens_Seen = 0u;
    }
    ImuErr = Sens_ImuErrCount;
    MagErr = Sens_MagErrCount;
    __enable_irq();

#if (SENS_PLOT_FIREWATER != 0)
    if (PlotDue != 0u)
    {
        Sens_PlotFrame(&Snapshot);
    }
#endif

    if (LogDue == 0u)
    {
        return;                 /* Nothing left this slot but a plot frame */
    }

    /* A failed transfer is reported here rather than where it happened, because
     * the tick has no console. One line per change of the total, so a sensor
     * that has come loose says so once instead of twice per tick. */
    if ((ImuErr + MagErr) != Sens_ErrLogged)
    {
        Sens_ErrLogged = ImuErr + MagErr;
        log_info("[SENS] bus read failures: imu=%d mag=%d\r\n",
                 (int)ImuErr, (int)MagErr);
    }

    /* While a window is open this line replaces the axes, on the same cadence:
     * the calibration module owns both the line and the state it reports. */
    if (App_SensorCalib_IsActive() != 0u)
    {
        App_SensorCalib_Report(Now);
        return;
    }

#if (LOG_PERIODIC_ENABLE != 0)
    /* One line per period, and only on a period in which some chain delivered:
     * the console carries the latest reading of each live chain and nothing
     * else, so a period with nothing new in it stays quiet rather than printing
     * a line to say it was quiet.
     *
     * Periodic, and therefore off by default -- see log_cfg.h. */
    if (Seen != 0u)
    {
        const char *pSep = "[SENS] ";

        if ((Seen & APP_SENSOR_HAVE_ACCEL) != 0u)
        {
            log_info("%saccel: X=%5dmg Y=%5dmg Z=%5dmg |a|=%5dmg",
                     pSep, (int)Snapshot.AccelMgX, (int)Snapshot.AccelMgY,
                     (int)Snapshot.AccelMgZ,
                     (int)Sens_Isqrt((uint32_t)((Snapshot.AccelMgX * Snapshot.AccelMgX) +
                                                (Snapshot.AccelMgY * Snapshot.AccelMgY) +
                                                (Snapshot.AccelMgZ * Snapshot.AccelMgZ))));
            pSep = "   ";
        }

        if ((Seen & APP_SENSOR_HAVE_GYRO) != 0u)
        {
            log_info("%sgyro: X=%6dmdps Y=%6dmdps Z=%6dmdps",
                     pSep, (int)Snapshot.GyroMdpsX, (int)Snapshot.GyroMdpsY,
                     (int)Snapshot.GyroMdpsZ);
            pSep = "   ";
        }

        if ((Seen & APP_SENSOR_HAVE_MAG) != 0u)
        {
            log_info("%smag: X=%5dmG Y=%5dmG Z=%5dmG",
                     pSep, (int)Snapshot.MagMgaussX, (int)Snapshot.MagMgaussY,
                     (int)Snapshot.MagMgaussZ);
        }

        log_info("\r\n");
    }
#endif /* LOG_PERIODIC_ENABLE */
}

const App_Sensor_Sample_t *App_Sensor_GetSample(void)
{
    return &Sens_Sample;
}

uint8_t App_Sensor_IsMagReady(void)
{
    return Sens_MagReady;
}
