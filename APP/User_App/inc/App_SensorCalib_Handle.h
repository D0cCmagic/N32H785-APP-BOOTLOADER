/**
 * @file    App_SensorCalib_Handle.h
 * @brief   Magnetometer hard-iron calibration: the key, the window and the fit.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.0.0
 *
 * @note    Split out of App_Sensor_Handle.h with no change of behaviour. This
 *          module owns the calibration button, the window state machine and the
 *          running per-axis min/max; the sampling chain it works from -- the I2C
 *          transactions, App_Sensor_UpdateData_Tick(), the timestamps, the failure counters
 *          and the console's axes line -- stayed in App_Sensor_Handle.c.
 *
 *          The two modules meet at App_Sensor_Sample_t: a magnetometer sample is
 *          handed over as it was read, and nothing here writes to that buffer.
 */

#ifndef _APP_SENSORCALIB_HANDLE_H_
#define _APP_SENSORCALIB_HANDLE_H_

#include <stdint.h>
#include "App_Sensor_Handle.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 1. Magnetometer hard-iron calibration.
 *
 *    The magnetometer reads 329..439 mG where China's total field is 480..500.
 *    Hard iron alone cannot explain that: an offset displaces the response
 *    sphere's centre while leaving its radius equal to |B_earth|. So it is the
 *    RADIUS, not the centre, that decides whether fusion is viable, and the
 *    calibration reports the two separately for exactly that reason.
 *
 *    Method: the key opens a window, the board is rotated through every
 *    orientation, and per-axis min/max accumulate over the magnetometer
 *    samples. At the end, offset = (min+max)/2 and radius = (max-min)/2. The
 *    radius is only trustworthy if the coverage was: rotating through a narrow
 *    cone under-reports it, because an axis that never turns end-on can never
 *    show the field's full projection onto it. SENS_MAG_CALIB_PTS_MIN exists
 *    to make thin coverage say so rather than pass silently.
 *
 *    Soft iron is not corrected here and cannot be, by this method: it turns
 *    the sphere into an ellipsoid, so the three radii come out unequal. They
 *    are logged per axis so that shows up, and the mean is a crude estimator
 *    whenever they disagree.
 *
 *    Nothing is applied to the live sample. This run measures, and correcting
 *    the reading needs the offsets stored somewhere -- a decision to take once
 *    the radius is known, because the answer decides whether it is worth it. */
#define SENS_MAG_CALIB_WINDOW_MS        (30000u)
#define SENS_MAG_CALIB_PTS_MIN          (100u)

/* Nominal total field for China. It varies widely across the country, roughly
 * 450 mG in the south to 540 mG in the north, so this is a reference to compare
 * against rather than a target to hit. */
#define SENS_MAG_EXPECTED_FIELD_MG      (490)

/* A mean radius within this percentage of the reference is called consistent
 * with it. The band is deliberately wide: the reference itself spans the
 * country, and the magnetometer sits next to a live radio, a microSD socket and
 * SDRAM, so a few percent of board-level field is expected. */
#define SENS_MAG_CALIB_TOL_PCT          (20)

/* Fallback for a key pad that turns out to be dead, which is a real risk on
 * PC0 -- it is on the same AFIO_ANAEN_CFG2 list as the sensor pin that failed.
 * Set to 1 and the same measurement runs with no button at all, after a delay
 * long enough to pick the board up. Default off, so the normal build carries
 * no extra code and no unused state. */
#define SENS_MAG_CALIB_AUTOSTART        (0)
#define SENS_MAG_CALIB_AUTOSTART_MS     (5000u)

/**
 * @name    App_SensorCalib_Init
 * @brief   Bring up the calibration key pad and report its idle level.
 * @param   None
 * @retval  None
 * @note    The key is brought up and reported first, ahead of any sensor work,
 *          by App_Sensor_Init(). A dead pad ends this feature before it starts,
 *          and the line is what turns "the button does nothing" into a named
 *          pin: an idle key must read released, so a level of 1 here means the
 *          pad never came up, the switch is stuck, or the board wires it the
 *          other way round.
 */
void App_SensorCalib_Init(void);

/**
 * @name    App_SensorCalib_Ready
 * @brief   Arm the delayed automatic calibration window.
 * @param   None
 * @retval  None
 * @note    Called at the end of App_Sensor_Init(), once the magnetometer is
 *          known to be up. Only the SENS_MAG_CALIB_AUTOSTART build has state to
 *          arm; with it off this is an empty call, which is what keeps the
 *          conditional out of the sampling module.
 */
void App_SensorCalib_Ready(void);

/**
 * @name    App_SensorCalib_Poll
 * @brief   Poll the calibration key, arm the delayed window and close the open
 *          one.
 * @param   Now: current TimeBase value.
 * @retval  None
 * @note    Called from App_Sensor_Poll() in the scheduler's 10 ms slot, which is
 *          what debounces the key: a tact switch settles well inside one poll
 *          period. It runs ahead of the console's gates in that slot so that a
 *          press is still seen while a calibration window is closing it.
 */
void App_SensorCalib_Poll(uint32_t Now);

/**
 * @name    App_SensorCalib_Report
 * @brief   Print one progress line for the open calibration window.
 * @param   Now: current TimeBase value.
 * @retval  None
 * @note    Same cadence as the console's axes line, which it replaces while a
 *          window is open: still one line per period, but it carries what the
 *          run needs watched -- elapsed time, how many points have landed, and
 *          the ranges so far, which is what shows whether the coverage is
 *          actually growing. Only meaningful while App_SensorCalib_IsActive()
 *          says a window is open.
 */
void App_SensorCalib_Report(uint32_t Now);

/**
 * @name    Sens_CalibAccumulate
 * @brief   Fold one magnetometer sample into the running per-axis min/max.
 * @param   pSample: the sample that was just read.
 * @retval  None
 * @note    Called from App_Sensor_UpdateData_Tick() with the sample the tick just decoded,
 *          and only while App_SensorCalib_IsActive() says a window is open. The
 *          name kept its original spelling so that the move out of
 *          App_Sensor_Handle.c stays a move and nothing else.
 * @note    The first point seeds min and max instead of being compared against
 *          zero. Comparing against a zeroed accumulator would drag every axis
 *          towards zero and inflate the radius, which is the one number this
 *          run exists to report.
 */
void Sens_CalibAccumulate(const App_Sensor_Sample_t *pSample);

/**
 * @name    App_SensorCalib_IsActive
 * @brief   Whether a calibration window is open.
 * @param   None
 * @retval  1 while the window is collecting samples, 0 otherwise.
 * @note    Read from the sampling tick as well as from the main loop: the tick
 *          stops folding samples in the moment the window closes, and the main
 *          loop swaps the console line over. The window itself is only opened
 *          and closed from the main loop, so the flag is written from one
 *          context and read from two.
 */
uint8_t App_SensorCalib_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_SENSORCALIB_HANDLE_H_ */
