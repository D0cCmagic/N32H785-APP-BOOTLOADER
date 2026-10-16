/**
 * @file    App_Sensor_Handle.h
 * @brief   The two I2C sensors: MMC5617WA magnetometer on one soft-I2C bus,
 *          LSM6DSV16X 6-axis IMU on the other.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.3.0
 *
 * @note    Application layer. Every byte on the wire goes through the
 *          Mcal_SensI2C_* primitives underneath; this file owns the device
 *          protocols, the unit conversion and the sampling policy.
 *
 *          Magnetometer calibration moved out unchanged: the key, the window
 *          and the fit are in App_SensorCalib_Handle.h now, and this module
 *          reaches them through App_SensorCalib_* calls.
 *
 *          One device per bus. The sensors are on flying leads, so a shared
 *          pair would put both breakout boards on one stub and let either
 *          one hold the whole bus down; a pair each also means neither scan
 *          can be confused by the other part.
 *
 *          Bring-up order is deliberate: prove the bus before proving the
 *          parts, and prove a read before making a write.
 *
 *          Sampling is separated from logging. App_Sensor_Poll() runs in the
 *          scheduler's 10 ms slot and fills App_Sensor_Sample_t, so every
 *          consumer reads the same values in the same units; the console line
 *          is emitted on its own much slower period.
 */

#ifndef APP_SENSOR_HANDLE_H
#define APP_SENSOR_HANDLE_H

#include <stdint.h>
#include "Mcal_SensI2C.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 1. Which bus carries which part. Single assignment point: the drivers take
 *    their bus from here rather than naming one inline. */
#define SENS_MAG_BUS                    SENS_I2C_BUS_0  /* PC6 = SCL, PC7 = SDA */
#define SENS_IMU_BUS                    SENS_I2C_BUS_1  /* PC11 = SCL, PC12 = SDA */

/* 2. The addresses this board is expected to answer on.
 *    The magnetometer address is fixed in silicon, the IMU picks between two
 *    by strapping SA0, and both are checked individually at init so the log
 *    says which part is missing rather than just "nothing found". */
#define SENS_MMC5617WA_ADDR7            (0x30u)
#define SENS_LSM6DSV16X_ADDR7_SA0_LOW   (0x6Au)
#define SENS_LSM6DSV16X_ADDR7_SA0_HIGH  (0x6Bu)

/* 3. Identity registers. One read per part proves the read transaction and the
 *    chip identity at the same time, which is why it comes before any
 *    configuration write: a failure here is a transfer problem, a failure
 *    after it could be either.
 *
 *    The magnetometer's Product ID is 0x12. The datasheet's register detail
 *    section prints that value, its register summary table is misaligned far
 *    enough to be unusable, and the same generation's MMC5603NJ answers 0x10.
 *    Hardware settled it: 0x12 is what this part reads back. */
#define SENS_MMC5617WA_REG_PRODUCT_ID   (0x39u)
#define SENS_MMC5617WA_PRODUCT_ID       (0x12u)
#define SENS_LSM6DSV16X_REG_WHO_AM_I    (0x0Fu)
#define SENS_LSM6DSV16X_WHO_AM_I        (0x70u)

/* 4. IMU accelerometer.
 *
 *    CTRL3 powers up as 0x44: BDU (bit 6) and IF_INC (bit 2) both set. IF_INC
 *    is what lets a single transfer walk 0x28..0x2D without re-sending the
 *    register address, so it is read back and reported before anything relies
 *    on it, and the datasheet lists both bits as defaulting to 1.
 *
 *    CTRL1 bit 7 must be 0, OP_MODE_XL[6:4] = 000 is high-performance mode and
 *    is the reset value, and ODR_XL[3:0] = 0110 is 120 Hz. Writing it is what
 *    takes the accelerometer out of power-down. No interrupt pin is routed and
 *    the FIFO is untouched. */
#define SENS_LSM6DSV16X_REG_CTRL1       (0x10u)
#define SENS_LSM6DSV16X_REG_CTRL3       (0x12u)
#define SENS_LSM6DSV16X_REG_STATUS      (0x1Eu)
#define SENS_LSM6DSV16X_REG_OUTX_L_A    (0x28u)

#define SENS_LSM6DSV16X_CTRL3_DEFAULT   (0x44u)  /* BDU = 1, IF_INC = 1         */
#define SENS_LSM6DSV16X_CTRL1_ODR_120HZ (0x06u)  /* High-performance, 120 Hz    */
#define SENS_LSM6DSV16X_STATUS_XLDA     (0x01u)  /* Accelerometer data ready    */

/* Six bytes, OUTX_L_A (0x28) through OUTZ_H_A (0x2D): three little-endian
 * signed 16-bit words, X then Y then Z. */
#define SENS_IMU_ACCEL_BYTES            (6u)

/* CTRL8 is left at its reset value, so the full scale is +-2 g. Two g spread
 * over the 32768 codes of a signed 16-bit word is 2000 mg, which puts one g at
 * exactly 1000 mg rather than the 999 that the datasheet's rounded
 * 0.061 mg/LSB figure would give. */
#define SENS_IMU_ACCEL_FS_MG            (2000)
#define SENS_IMU_ACCEL_MG(raw)          (((int32_t)(int16_t)(raw) * SENS_IMU_ACCEL_FS_MG) / 32768)

/* 5. IMU gyroscope. Same shape as the accelerometer: one range and one rate
 *    write, each proven by a read-back, then the three axes.
 *
 *    The range is raised off its reset value on purpose. CTRL6 powers up at
 *    +-125 dps, which a hand rotating the board will reach and exceed, and a
 *    saturated axis reads a constant full-scale value -- indistinguishable
 *    from a stuck driver. +-500 dps covers ordinary handling while still
 *    resolving 17.5 mdps per count.
 *
 *    ODR_G uses the same code as ODR_XL: 0110 is 120 Hz, in this case in
 *    low-power/high-performance mode, with OP_MODE_G[6:4] left at its
 *    high-performance default. */
#define SENS_LSM6DSV16X_REG_CTRL2       (0x11u)
#define SENS_LSM6DSV16X_REG_CTRL6       (0x15u)
#define SENS_LSM6DSV16X_REG_OUTX_L_G    (0x22u)

#define SENS_LSM6DSV16X_CTRL2_ODR_120HZ (0x06u)  /* High-performance, 120 Hz    */
#define SENS_LSM6DSV16X_CTRL6_FS_500DPS (0x02u)  /* FS_G[3:0] = 0010            */
#define SENS_LSM6DSV16X_STATUS_GDA      (0x02u)  /* Gyroscope data ready, bit 1 */

/* OUTX_L_G (0x22) through OUTZ_H_G (0x27), laid out exactly like the
 * accelerometer's six bytes */
#define SENS_IMU_GYRO_BYTES             (6u)

/* The two chains are adjacent in the register map: the accelerometer's first
 * byte is at 0x28, immediately after the gyroscope's last byte at 0x27. One
 * transfer from OUTX_L_G therefore returns both chains in twelve bytes, with
 * the gyroscope's XYZ first and the accelerometer's second. IF_INC is what
 * lets the part walk that far without re-sending an address, and that order is
 * the only thing keeping the two chains apart -- the stream carries no tag. */
#define SENS_IMU_BURST_BYTES            (12u)

/* 17.50 mdps/LSB at +-500 dps, which is exactly 35/2 -- no decimal needed. The
 * datasheet's sensitivity is the specified figure, not the nominal full-scale
 * ratio, so this is the number to convert with. */
#define SENS_IMU_GYRO_MDPS(raw)         (((int32_t)(int16_t)(raw) * 35) / 2)

/* 6. Magnetometer.
 *
 *    Reading this datasheet is not like reading the ST one. Its register
 *    SUMMARY table is misaligned badly enough to be unusable, and so is the
 *    bit-name column of several register DETAIL tables -- in Control 2 the
 *    names and their descriptions cannot be paired up at all, which is why
 *    continuous mode is not used here. `pdftotext -raw` preserves reading
 *    order and does line the bit rows up; anything else does not. Treat the
 *    plain and -layout extractions as unreliable for this part.
 *
 *    The path taken instead is the one the datasheet's own I2C communication
 *    example spells out: write CTRL0 with Auto_SR_en and Take_meas_M set, and
 *    the part performs one magnetic measurement. No continuous mode, no ODR,
 *    and no guessing at an ambiguous bit.
 *
 *    Two properties of the output that are easy to get wrong:
 *
 *      1. It is UNSIGNED, not two's complement. Zero field sits near 2^19, so
 *         a small field is a small deviation from the middle of the range, not
 *         a small number.
 *      2. Each axis is 20 bits split across three bytes, and register 0x06
 *         carries the low nibble of two axes at once -- X in its high half, Y
 *         in its low half. */
#define SENS_MMC5617WA_REG_XOUT0        (0x00u)
#define SENS_MMC5617WA_REG_XYOUT2       (0x06u)
#define SENS_MMC5617WA_REG_ZOUT2        (0x07u)
#define SENS_MMC5617WA_REG_STATUS       (0x18u)
/* The rate register is at 0x1A on this part. It is not the MMC5603's map:
 * 0x1B/0x1C/0x1D are control 0/1/2 here, and 0x1D is not the rate. */
#define SENS_MMC5617WA_REG_ODR          (0x1Au)
#define SENS_MMC5617WA_REG_CTRL0        (0x1Bu)
#define SENS_MMC5617WA_REG_CTRL1        (0x1Cu)
#define SENS_MMC5617WA_REG_CTRL2        (0x1Du)

#define SENS_MMC5617WA_CTRL0_AUTO_SR    (0x20u)  /* Auto_SR_en, bit 5          */
/* Cmm_freq_en is a command, not a mode bit: writing it starts the part's
 * internal calculation of the counter target, and the part clears it again once
 * that finishes. Read the register back straight away and bit 7 is already
 * gone, so a check that demands it stick fails a part that is working
 * perfectly. Auto_SR_en is the only bit of the two that persists. */
#define SENS_MMC5617WA_CTRL0_CMM_FREQ   (0x80u)  /* Cmm_freq_en, bit 7         */
#define SENS_MMC5617WA_CTRL2_CMM_EN     (0x10u)  /* Cmm_en, bit 4              */

/* Control register 1's BW1:BW0 set the decimation length, and with it both the
 * duration of one measurement and the fastest the part will run while
 * automatic set/reset is on: 00 is 6.6 ms and a 75 Hz ceiling, 01 is 3.5 ms
 * and 125 Hz, 10 is 2.0 ms and 255 Hz. 00 cannot reach the 100 Hz the poll
 * wants, and 01 would sit at four fifths of its own ceiling, so 10 it is.
 *
 * Continuous mode takes three writes, in this order: ODR first and non-zero,
 * or the mode will not start at all, then Cmm_freq_en so the part can work out
 * its counter reload, then Cmm_en to start it. Auto_SR_en stays set -- it
 * applies to continuous measurements as much as to single ones. */
#define SENS_MMC5617WA_CTRL1_BW10       (0x02u)  /* BW1:BW0 = 10, 2.0 ms       */
#define SENS_MMC5617WA_ODR_HZ           (100u)   /* one measurement per 10 ms  */

#define SENS_MMC5617WA_STATUS_MEAS_M    (0x40u)  /* Meas_m_done, bit 6         */

/* Xout0 through Zout2 in one burst: 0x00..0x07. Reading any magnetic data
 * register is also what clears Meas_m_done. */
#define SENS_MAG_DATA_BYTES             (8u)

/* The output is unsigned, so zero field sits near the middle of the 20-bit
 * range rather than at zero */
#define SENS_MMC5617WA_ZERO_COUNT       (524288)

/* 16384 counts/G, so 1000/16384 mG per count, which is exactly 125/2048. The
 * exact ratio keeps the arithmetic integer and avoids a repeating decimal. */
#define SENS_MMC5617WA_MG(count)        ((((int32_t)(count) - SENS_MMC5617WA_ZERO_COUNT) * 125) / 2048)

/* 7. Cadence.
 *
 *    The sampling runs in the base tick's interrupt, on the division of it that
 *    Mcal_TimerTask owns: MCAL_SAMPLE_TICK_HZ is 2000 and MCAL_SAMPLE_DIV is
 *    16, so App_Sensor_UpdateData_Tick() is called at 125 Hz. That is deliberately a
 *    little faster than the IMU's 120 Hz output rate rather than equal to it: a
 *    reader slower than the part loses a sample every time the two periods
 *    drift past each other, while a reader faster than the part never does --
 *    it finds nothing new on the odd tick, and the sample flags say so. The
 *    microsecond clock is that tick counted rather than measured, because the
 *    tick is an exact division of the clock.
 *
 *    The magnetometer is read on a division of the sample rate rather than on
 *    every tick, which is why it runs continuously at all: one conversion takes
 *    2 ms at BW=10, and a triggered measurement could not have been asked for
 *    often enough to keep up. The field turns far more slowly than the body
 *    does, so reading it at SENS_MAG_DIV of the sample rate costs the filter
 *    nothing it can measure and keeps the interrupt short.
 *
 *    SENS_LOG_PERIOD_MS is the console policy: one rate line per period, the
 *    same cadence the radio statistics use so the two do not interleave into a
 *    wall of text. That line reports p, the main loop's own slot count, next to
 *    a, m and t, which are what the tick measured -- so the rates are read
 *    rather than assumed. */
#define SENS_MAG_DIV                    (4u)
/* How long Sens_MagConfig() gives the part to start, used twice: as the
 * deadline for the Cmm_freq_en handshake, where expiring only means the
 * calculation took longer than expected and the mode is tried anyway, and then
 * as the wait for the first measurement, where expiring is a failure. The
 * second use is the one that matters: three control registers can read back
 * exactly as written while the part measures nothing, and that is worth
 * catching at init rather than showing up later as a magnetometer that never
 * delivers a sample. Ten ODR periods leaves room for the set/reset overhead. */
#define SENS_MAG_STARTUP_MS             (100u)
#define SENS_LOG_PERIOD_MS              (5000u)

/* 8. Which members of App_Sensor_Sample_t hold data from the tick that
 *    produced TickMs and TickUs. A chain that never answered at init never sets
 *    its bit at all, so a consumer can still tell "this part is not fitted"
 *    from "this part read zero". A fitted part that misses a tick is normal,
 *    though: the IMU runs at a slightly different rate from the read, and the
 *    magnetometer is read on a division of that read, so a tick now and then
 *    finds no new sample -- and the absent bit is how a consumer knows not to
 *    treat the value it is holding as new. */
#define APP_SENSOR_HAVE_ACCEL           (0x01u)
#define APP_SENSOR_HAVE_GYRO            (0x02u)
#define APP_SENSOR_HAVE_MAG             (0x04u)

/* 9. Magnetometer hard-iron calibration moved out: the thresholds, the key
 *    fallback and the whole window state machine are in
 *    App_SensorCalib_Handle.h. */

/* 10. Plot output.
 *
 *     VOFA+ reads FireWater as one frame per line: the channel values as ASCII
 *     numbers separated by commas and terminated by a newline. Set this to 1 to
 *     watch the sensors as waveforms instead of reading them as numbers.
 *
 *     The channel order is fixed by position, because that is how the receiver
 *     keys them: three acceleration in mg, three rates in mdps, three field
 *     components in mG. All nine go out in every frame whether or not that
 *     chain produced anything since the last one -- the field changes far too
 *     slowly to be worth a waveform, but dropping it would renumber everything
 *     after it.
 *
 *     The period is bounded by the console, not by the sample tick. The log UART
 *     writes one character at a time in a busy loop, so a nine-channel frame of
 *     about 60 characters costs about 5 ms of blocked main loop at the 115200
 *     the console runs at -- which is why 100 here is 10 Hz and not the 100 Hz
 *     the sample tick could feed. Raising the console to 921600 makes the same
 *     frame about 0.66 ms, and this can then be 10: one frame per slot, against
 *     a 125 Hz sample tick underneath, so anything faster would print the same
 *     sample twice rather than show anything new.
 *
 *     Off by default, so a build nobody is watching keeps the two text lines and
 *     nothing else: the frame and everything it needs are compiled out rather
 *     than switched off. The text lines are not suppressed while it is on: a
 *     receiver treats a line it cannot parse as one bad frame and carries on,
 *     and one of those every five seconds is cheaper than losing them here. */
#define SENS_PLOT_FIREWATER             (0)
#define SENS_PLOT_PERIOD_MS             (100u)

/**
 * @brief  The latest sample from all three measurement chains, in engineering
 *         units.
 *
 * @note   The member names carry their unit, and they have to: mg is milli-g
 *         and mG is milli-gauss, one letter apart and three orders of magnitude
 *         apart in meaning, so the magnetometer's members spell the unit out.
 *         Every value is a signed 32-bit count of that unit -- there is no
 *         float and no fixed point anywhere in this module.
 *
 *         TickMs is the TimeBase value at the moment the sample was taken, not
 *         at the moment it was read out: the console line can lag the data by
 *         seconds, and a consumer that needs to know how old a value is has to
 *         subtract from this rather than from its own clock.
 *
 *         TickUs is the same instant on the sampling tick's own microsecond
 *         clock, and it is the one the filter integrates over. It is counted
 *         from the tick rather than read from a free-running timer, so its
 *         resolution is exactly the tick's period and it cannot wrap inside a
 *         session; a difference of two of them is exact to that resolution.
 */
typedef struct
{
    int32_t  AccelMgX;              /* milli-g, at rest 1000 on the up axis     */
    int32_t  AccelMgY;              /* milli-g                                  */
    int32_t  AccelMgZ;              /* milli-g                                  */
    int32_t  GyroMdpsX;             /* milli-degrees per second, 0 at rest      */
    int32_t  GyroMdpsY;             /* milli-degrees per second                 */
    int32_t  GyroMdpsZ;             /* milli-degrees per second                 */
    int32_t  MagMgaussX;            /* milli-gauss                              */
    int32_t  MagMgaussY;            /* milli-gauss                              */
    int32_t  MagMgaussZ;            /* milli-gauss                              */
    uint32_t TickMs;                /* TimeBase when this sample was taken      */
    uint32_t TickUs;                /* sampling tick's microseconds, x 1e-6 s   */
    uint8_t  Flags;                 /* APP_SENSOR_HAVE_*, new in THIS tick      */
} App_Sensor_Sample_t;

/**
 * @brief  Driver return codes.
 */
typedef enum
{
    SENS_OK = 0,                    /* Everything answered                      */
    SENS_ERR_BUS = -1,              /* A soft-I2C bus would not come up         */
    SENS_ERR_SCAN = -2,             /* Scan itself failed, bus fault            */
    SENS_ERR_MAG_MISSING = -3,      /* No answer at the magnetometer address    */
    SENS_ERR_IMU_MISSING = -4,      /* No answer at either IMU address          */
    SENS_ERR_ID = -5,               /* Identity register could not be read      */
    SENS_ERR_IMU_CONFIG = -6,       /* A register written to the IMU did not    */
                                    /* read back as written                     */
    SENS_ERR_MAG_CONFIG = -7        /* The same, for the magnetometer           */
} App_Sensor_Status_t;

/**
 * @name    App_Sensor_Init
 * @brief   Bring up both soft-I2C buses, scan each one, check that the part
 *          expected on it answered, then configure both parts and start the
 *          magnetometer's first measurement.
 * @param   None
 * @retval  SENS_OK when both parts answered and both took their configuration,
 *          a negative App_Sensor_Status_t otherwise. The console output is the
 *          real result: it lists every address found on each bus and the value
 *          each register read back, whatever the return code.
 * @note    A non-zero return does not stop the firmware. Sampling is skipped
 *          for any part that did not come up, so a dead sensor costs its own
 *          readings and nothing else.
 */
int App_Sensor_Init(void);

/**
 * @name    App_Sensor_UpdateData_Tick
 * @brief   Read every chain that has a fresh sample and stamp what landed with
 *          the sampling tick's own microsecond clock.
 * @param   None
 * @retval  1 when at least one chain produced new data, 0 when none had, which
 *          is the tick the filter must not step on.
 * @note    Called from the base tick's interrupt at the sample rate, which
 *          makes this the only writer of the sample buffer. The filter that
 *          consumes it runs in the same context, so nothing on this path needs
 *          a critical section.
 * @note    Nothing in here may print. The console waits on the transmitter one
 *          character at a time, and this runs where the radio and the network
 *          are already waiting, so a failed transfer is counted and left for
 *          the rate line to report.
 * @note    A bus fault backs the whole step off for one log period, for the
 *          same reason the poll used to: the bit-bang loops spin to their
 *          timeout before giving up, which is far longer than this tick.
 */
int App_Sensor_UpdateData_Tick(void);

/**
 * @name    App_Sensor_Poll
 * @brief   Print the rates the sampling tick measured, and give the slot to the
 *          calibration's own poll.
 * @param   None
 * @retval  None
 * @note    Called from the scheduler's 10 ms slot. The sampling itself is not
 *          here any more: it runs in the base tick's interrupt, which is what
 *          makes its rate the timer's rather than this slot's. What this slot
 *          still owns is the console, which waits on the transmitter, and the
 *          hand-off to App_SensorCalib_Poll(), which debounces the calibration
 *          key against this same cadence.
 * @note    While a calibration window is open the console's rate line is
 *          replaced by App_SensorCalib_Report() on the same period, which is
 *          why the poll asks App_SensorCalib_IsActive() before printing.
 */
void App_Sensor_Poll(void);

/**
 * @name    App_Sensor_GetSample
 * @brief   The most recent sample, for any consumer that is not the console.
 * @param   None
 * @retval  Pointer to the module's one sample buffer. Never NULL once
 *          App_Sensor_Init() has run. Flags holds the chains that delivered
 *          data in the tick that produced TickMs, so a consumer can tell a
 *          fresh value from the one before it.
 * @note    The buffer is written in place by the sampling tick's interrupt, so
 *          a consumer in the main loop can be reading it when it changes: copy
 *          what is needed with interrupts masked, or read it from inside the
 *          tick. The filter does the latter, which is why taking this pointer
 *          is safe there and only there.
 */
const App_Sensor_Sample_t *App_Sensor_GetSample(void);

/**
 * @name    App_Sensor_IsMagReady
 * @brief   Whether the magnetometer came up and was configured at init.
 * @param   None
 * @retval  1 when the field is being read, 0 when the part never answered.
 * @note    The sampling chain keeps this to itself; it is exposed because the
 *          calibration module refuses to open a window without it, so that a
 *          press on a dead board answers with a reason rather than thirty
 *          seconds of nothing.
 */
uint8_t App_Sensor_IsMagReady(void);

/**
 * @name    Sens_Isqrt
 * @brief   Integer square root, used for the magnitude of the acceleration
 *          vector.
 * @param   Value: value to take the root of.
 * @retval  Floor of the square root.
 * @note    Nothing else in this firmware links the floating point library, and
 *          one slow log line is a poor reason to start: a bit-by-bit root is
 *          a dozen integer instructions and is exact.
 * @note    No longer static, and the name is unchanged: the calibration module
 *          needs the same root for the hard-iron offset magnitude, so it is
 *          declared here. The definition stays in App_Sensor_Handle.c.
 */
uint32_t Sens_Isqrt(uint32_t Value);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_HANDLE_H */
