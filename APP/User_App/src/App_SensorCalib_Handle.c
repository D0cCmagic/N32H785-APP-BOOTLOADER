/**
 * @file    App_SensorCalib_Handle.c
 * @brief   Magnetometer hard-iron calibration: the key, the window and the fit.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.0.0
 *
 * @note    Split out of App_Sensor_Handle.c with no change of behaviour: every
 *          line below was moved, not rewritten. What stayed behind is the
 *          sampling chain -- the I2C transactions, App_Sensor_UpdateData_Tick(), the
 *          timestamps, the failure counters and the console's axes line.
 *
 *          See App_SensorCalib_Handle.h for the method and the thresholds, and
 *          App_Sensor_Handle.h for the units the samples arrive in.
 *
 *          Nothing below this file knows a register address or a scale factor:
 *          the field arrives as milli-gauss in App_Sensor_Sample_t, and the
 *          offsets this run measures are reported, not applied.
 */

#include "App_SensorCalib_Handle.h"
#include "App_Sensor_Handle.h"
#include "log.h"
#include "Mcal_KeyGpio_Init.h"
#include "main.h"                   /* provides extern volatile uint32_t TimeBase */

/* Calibration state. Deliberately plain: three running min/max pairs and a
 * count, every one of which the console line prints, so the console cannot
 * disagree with what the fit was computed from. */
static uint8_t  Sens_CalibActive = 0u;
static uint8_t  Sens_CalibKeyPrev = 0u;
static uint32_t Sens_CalibStartTick = 0u;
static uint32_t Sens_CalibPoints = 0u;
static int32_t  Sens_CalibMin[3];
static int32_t  Sens_CalibMax[3];
#if (SENS_MAG_CALIB_AUTOSTART != 0)
static uint8_t  Sens_CalibArmed = 0u;
static uint32_t Sens_CalibArmTick = 0u;
#endif

static void Sens_CalibKeyPoll(uint32_t Now);
static void Sens_CalibStart(uint32_t Now);
static void Sens_CalibFinish(uint32_t Now);

void App_SensorCalib_Init(void)
{
    /* The key is brought up and reported first, ahead of any sensor work. A
     * dead pad ends this feature before it starts, and this line is what turns
     * "the button does nothing" into a named pin: an idle key must read
     * released, so a level of 1 here means the pad never came up, the switch is
     * stuck, or the board wires it the other way round. */
    Mcal_KeyGpio_Init();
    log_info("[SENS] key %s idle level = %d, %s\r\n", KEY_CALIB_NAME,
             (int)Key_GetLevel(),
             (Key_IsPressed() == 0u) ? "released" : "READS PRESSED AT IDLE");
}

/**
 * @name    Sens_CalibStart
 * @brief   Open a calibration window and begin collecting samples.
 * @param   Now: current TimeBase value.
 * @retval  None
 * @note    A window is refused when the magnetometer never came up, so that a
 *          press on a dead board answers with a reason rather than thirty
 *          seconds of nothing.
 */
static void Sens_CalibStart(uint32_t Now)
{
    if (App_Sensor_IsMagReady() == 0u)
    {
        log_info("[SENS] mag calib: magnetometer is not up, nothing to collect\r\n");
        return;
    }

    Sens_CalibActive    = 1u;
    Sens_CalibStartTick = Now;
    Sens_CalibPoints    = 0u;
    Sens_CalibMin[0]    = 0;
    Sens_CalibMin[1]    = 0;
    Sens_CalibMin[2]    = 0;
    Sens_CalibMax[0]    = 0;
    Sens_CalibMax[1]    = 0;
    Sens_CalibMax[2]    = 0;

    log_info("[SENS] mag calib start: rotate the board through every orientation, "
             "%d s, or press again to stop early\r\n",
             (int)(SENS_MAG_CALIB_WINDOW_MS / 1000u));
}

void Sens_CalibAccumulate(const App_Sensor_Sample_t *pSample)
{
    const int32_t Axis[3] = { pSample->MagMgaussX, pSample->MagMgaussY,
                              pSample->MagMgaussZ };
    uint8_t i;

    for (i = 0u; i < 3u; i++)
    {
        if (Sens_CalibPoints == 0u)
        {
            Sens_CalibMin[i] = Axis[i];
            Sens_CalibMax[i] = Axis[i];
        }
        else
        {
            if (Axis[i] < Sens_CalibMin[i])
            {
                Sens_CalibMin[i] = Axis[i];
            }
            if (Axis[i] > Sens_CalibMax[i])
            {
                Sens_CalibMax[i] = Axis[i];
            }
        }
    }

    Sens_CalibPoints++;
}

/**
 * @name    Sens_CalibFinish
 * @brief   Close the window and report the hard-iron fit.
 * @param   Now: current TimeBase value.
 * @retval  None
 * @note    offset = (min+max)/2 is the hard-iron estimate and radius =
 *          (max-min)/2 is the measured field. They are reported apart because
 *          only the radius can tell the earlier short reading apart from
 *          something no offset correction can fix: hard iron moves the centre,
 *          it cannot shrink the sphere.
 * @note    The three radii are printed individually before they are averaged.
 *          When they disagree the response is an ellipsoid rather than a
 *          sphere -- soft iron -- and the mean is a weak summary of that, which
 *          is why the per-axis numbers are the ones to read first.
 */
static void Sens_CalibFinish(uint32_t Now)
{
    int32_t  Off[3];
    int32_t  Rad[3];
    uint32_t RadSum;
    uint32_t Mean;
    uint32_t TolMg;
    uint8_t  i;

    Sens_CalibActive = 0u;

    if (Sens_CalibPoints == 0u)
    {
        log_info("[SENS] mag calib: no magnetometer samples were collected\r\n");
        return;
    }

    for (i = 0u; i < 3u; i++)
    {
        Off[i] = (Sens_CalibMin[i] + Sens_CalibMax[i]) / 2;
        Rad[i] = (Sens_CalibMax[i] - Sens_CalibMin[i]) / 2;
    }

    RadSum = (uint32_t)Rad[0] + (uint32_t)Rad[1] + (uint32_t)Rad[2];
    Mean   = RadSum / 3u;

    log_info("[SENS] mag calib done, %d pts in %d ms\r\n",
             (int)Sens_CalibPoints, (int)(Now - Sens_CalibStartTick));

    if (Sens_CalibPoints < SENS_MAG_CALIB_PTS_MIN)
    {
        log_info("[SENS] mag calib: only %d pts, expected at least %d -- the radii "
                 "below are a lower bound\r\n",
                 (int)Sens_CalibPoints, (int)SENS_MAG_CALIB_PTS_MIN);
    }

    for (i = 0u; i < 3u; i++)
    {
        log_info("[SENS] mag calib %c: min=%6d max=%6d offset=%6d radius=%6d mG\r\n",
                 (char)('X' + (int)i), (int)Sens_CalibMin[i], (int)Sens_CalibMax[i],
                 (int)Off[i], (int)Rad[i]);
    }

    log_info("[SENS] mag calib |B| radius mean = %d mG, reference ~%d mG (%d%%)\r\n",
             (int)Mean, (int)SENS_MAG_EXPECTED_FIELD_MG,
             (int)((Mean * 100u) / (uint32_t)SENS_MAG_EXPECTED_FIELD_MG));

    log_info("[SENS] mag calib hard-iron offset magnitude = %d mG\r\n",
             (int)Sens_Isqrt((uint32_t)((Off[0] * Off[0]) + (Off[1] * Off[1]) +
                                        (Off[2] * Off[2]))));

    TolMg = ((uint32_t)SENS_MAG_EXPECTED_FIELD_MG * SENS_MAG_CALIB_TOL_PCT) / 100u;

    if ((Mean + TolMg) < (uint32_t)SENS_MAG_EXPECTED_FIELD_MG)
    {
        log_info("[SENS] mag calib: radius is SHORT of the reference by more than "
                 "%d%% -- hard iron cannot do that, so the field or the scale is in "
                 "question\r\n", (int)SENS_MAG_CALIB_TOL_PCT);
    }
    else
    {
        log_info("[SENS] mag calib: radius is within %d%% of the reference, so the "
                 "short reading was hard iron and an offset fit addresses it\r\n",
                 (int)SENS_MAG_CALIB_TOL_PCT);
    }
}

/**
 * @name    Sens_CalibKeyPoll
 * @brief   Edge-detect the calibration key and open or close a window.
 * @param   Now: current TimeBase value.
 * @retval  None
 * @note    Called from the 10 ms poll, which is what debounces it: a tact
 *          switch settles well inside one poll period.
 * @note    A second press closes the window early. Running the full window is
 *          not a goal in itself -- coverage is -- and waiting out the remainder
 *          after every orientation has been visited only wastes the session.
 */
static void Sens_CalibKeyPoll(uint32_t Now)
{
    uint8_t Pressed = Key_IsPressed();

    if (Pressed == Sens_CalibKeyPrev)
    {
        return;
    }
    Sens_CalibKeyPrev = Pressed;

    if (Pressed == 0u)
    {
        return;                     /* Release carries no action */
    }

    if (Sens_CalibActive != 0u)
    {
        Sens_CalibFinish(Now);
    }
    else
    {
        Sens_CalibStart(Now);
    }
}

uint8_t App_SensorCalib_IsActive(void)
{
    return Sens_CalibActive;
}

void App_SensorCalib_Ready(void)
{
#if (SENS_MAG_CALIB_AUTOSTART != 0)
    Sens_CalibArmed   = 1u;
    Sens_CalibArmTick = TimeBase;
#endif
}

void App_SensorCalib_Poll(uint32_t Now)
{
    Sens_CalibKeyPoll(Now);

#if (SENS_MAG_CALIB_AUTOSTART != 0)
    if ((Sens_CalibArmed != 0u) &&
        ((Now - Sens_CalibArmTick) >= SENS_MAG_CALIB_AUTOSTART_MS))
    {
        Sens_CalibArmed = 0u;
        Sens_CalibStart(Now);
    }
#endif

    /* Checked after the reads so the last sample before the deadline still
     * counts. A press also closes the window, from Sens_CalibKeyPoll(). */
    if ((Sens_CalibActive != 0u) &&
        ((Now - Sens_CalibStartTick) >= SENS_MAG_CALIB_WINDOW_MS))
    {
        Sens_CalibFinish(Now);
    }
}

void App_SensorCalib_Report(uint32_t Now)
{
    log_info("[SENS] mag calib %d s, %d pts, X=%d..%d Y=%d..%d Z=%d..%d\r\n",
             (int)((Now - Sens_CalibStartTick) / 1000u), (int)Sens_CalibPoints,
             (int)Sens_CalibMin[0], (int)Sens_CalibMax[0],
             (int)Sens_CalibMin[1], (int)Sens_CalibMax[1],
             (int)Sens_CalibMin[2], (int)Sens_CalibMax[2]);
}
