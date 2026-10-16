/**
 * @file    App_Fusion_Handle.h
 * @brief   Attitude from the IMU and the magnetometer: a 7-state EKF over a
 *          quaternion and a gyro rate bias.
 * @author  D0cC
 * @date    2026-09-30
 * @version V0.4.0
 *
 * @note    Application layer, one step above App_Sensor_Handle. It reads the
 *          one sample buffer that module fills and never touches a bus, a
 *          register or a raw count: the readings arrive here already in
 *          milli-g, milli-degrees per second and milli-gauss.
 *
 *          The console side moved out unchanged: the queue of pending messages,
 *          the message ids and the lines printed for them are in
 *          App_FusionLog_Handle.h now, and this module raises a line through
 *          App_FusionLog_Post() when it has something to report. Nothing here
 *          prints, which is what lets App_Fusion_Update() run in the tick
 *          interrupt.
 *
 *          Frames. The body frame is FRD -- X forward, Y right, Z down -- and
 *          the nav frame is NED, but it is not magnetic north. Nav X is the
 *          body's own horizontal heading at the moment the startup reference
 *          was taken, so yaw reads zero at power-on and measures rotation since
 *          then. That choice is what makes the magnetometer's reference
 *          direction a vector computed from the startup attitude rather than
 *          the constant [mh, 0, mv] a textbook would use: that constant is only
 *          correct if nav X is magnetic north, and here it is not.
 *
 *          The state is seven numbers: four for the quaternion that takes body
 *          to nav, three for the gyro rate bias in the body frame. The bias is
 *          carried in the state rather than measured once and subtracted,
 *          because it moves with temperature, and a bias the filter estimates
 *          is a bias it keeps correcting.
 *
 *          Three conventions the theory note fixes and this module keeps: the
 *          quaternion is normalised after every operation that touches it, the
 *          matrix inverse in the correction step is taken by Cholesky
 *          factorisation rather than by a general inverse, and the predict step
 *          is a second-order Runge-Kutta step, because a first-order one is not
 *          accurate enough at this step size.
 *
 *          Time comes from the sample's own timestamp, never from a constant.
 *          The sampling runs on a hardware tick divided by a counter, so the
 *          nominal interval is exact -- but the IMU's own output rate is not the
 *          tick's, so the filter gets a fresh reading on most ticks and none on
 *          the rest. The interval between two samples the filter actually
 *          consumed is therefore the only honest integration step, and it is
 *          what App_Fusion_Update() integrates over.
 */

#ifndef APP_FUSION_HANDLE_H
#define APP_FUSION_HANDLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 1. Timing: the startup window, and how often the console hears from this
 *    module.
 *
 *    Before the first prediction the filter needs three things -- the gyro's
 *    rate bias, the attitude to start from, and the field direction that
 *    attitude implies -- and all three come out of one stretch of samples: the
 *    first APP_FUSION_BIAS_SAMPLES of them that arrive without the part being
 *    moved. A sample that shows motion empties the window and starts it again,
 *    so a board that gets knocked while booting still ends up with a reference
 *    taken from a quiet stretch.
 *
 *    The stall test covers the other failure. If no sample arrives for
 *    APP_FUSION_BIAS_STALL_MS the module says so once and keeps waiting, which
 *    is the honest behaviour for two different situations that look alike from
 *    here: a board that is still being handled, and an IMU whose bus never came
 *    up. Waiting rather than giving up means a sensor that starts answering
 *    late still gets a reference. Until then Flags is clear, so no consumer can
 *    mistake a level default for a measurement -- and an uncorrected gyro
 *    carrying a degree per second of offset is a yaw that walks, which is worth
 *    a line on the console either way. */
#define APP_FUSION_BIAS_SAMPLES         (100u)      /* 1 s at the nominal rate */
#define APP_FUSION_BIAS_MAX_DPS         (30)        /* above this it is moving */
#define APP_FUSION_BIAS_STALL_MS        (2000u)     /* no sample for this long */
#define APP_FUSION_LOG_PERIOD_MS        (5000u)     /* one console line        */

/* 2. Noise, and the covariance to start from.
 *
 *    Q and R are the two dials this filter is tuned with. Q is a rate, not a
 *    per-step amount, because the step is measured and irregular and a density
 *    is the only form that stays correct when the step changes; the per-step
 *    matrix is the density times the step. R is not a rate -- it is the
 *    variance of one normalised measurement, which is why its numbers look
 *    small and have no time in them at all.
 *
 *    R_ACC is measured rather than guessed, and the residual line is what
 *    measured it. The innovation covariance is H P H' + R, and H P H' only adds,
 *    so S >= R and the residual can never sit below sqrt(R_ACC_VAR) whatever the
 *    filter does. On the board it held at 2 milli-radians across nine
 *    consecutive periods, which no value above 4.0e-6 can produce; that is where
 *    this number comes from, and it agrees with the sensor line's own lateral
 *    scatter of 1-2 mg out of 1000. The 4.0e-4 it replaces was 100x too large,
 *    which told the filter the accelerometer was far worse than it is and made
 *    every correction weaker than intended -- without ever looking broken at
 *    rest, which is why the residual earns its place on the console.
 *
 *    R_MAG was settled by the same inference, on the 2026-09-29 static run, and
 *    came out too large. The magnetometer residual averaged 14 milli-radians
 *    across that run against a floor of sqrt(2.5e-3) = 50, which that value
 *    cannot produce. Fourteen is the length of a three-vector, so the per-axis
 *    figure is 14/1.6 = 9 milli-radians and the variance belongs near 8.0e-5;
 *    the field's direction, measured against gravity over the same run and with
 *    no attitude in the arithmetic, wandered by 0.8 degrees, which is the same
 *    answer. 1.5e-4 is that with a little margin, because the residual cannot
 *    separate the part's own noise from how much the two breakout boards move
 *    against each other. The 2.5e-3 it replaces was 17x too large.
 *
 *    Both sets of rows assume what they cannot check: the accelerometer rows
 *    assume a body that is mostly not accelerating, and the magnetometer rows
 *    assume a field that has been through nothing but the hard-iron offset,
 *    which this module does not estimate -- and, worse, that the field and the
 *    gravity it is compared against are expressed in one body frame, which the
 *    alignment matrices above are the only thing making true. */
/* The longest step the filter will integrate in one go. The step itself is not
 * a constant and there is deliberately no nominal to fall back on -- it is the
 * gap between the two samples the filter consumed, every time -- but a gap
 * longer than this is a main loop that stalled, and a console line is enough to
 * stall it. Clamping under-integrates and the next correction pulls that back;
 * taking the gap at face value, with the rate read after it, would swing the
 * attitude by whatever the stall happened to contain. */
#define APP_FUSION_DT_MAX               (0.1f)      /* s, ten nominal steps    */
#define APP_FUSION_Q_ATT_VAR            (1.0e-4f)   /* (rad/s)^2 per s         */
#define APP_FUSION_Q_BIAS_VAR           (1.0e-6f)   /* (rad/s^2)^2 per s       */
#define APP_FUSION_R_ACC_VAR            (4.0e-6f)   /* g^2, normalised         */
#define APP_FUSION_R_MAG_VAR            (1.5e-4f)   /* unit^2, normalised      */
#define APP_FUSION_P0_ATT_VAR           (1.0e-2f)   /* rad^2, about 6 degrees  */
#define APP_FUSION_P0_BIAS_VAR          (1.0e-3f)   /* (rad/s)^2               */

/* 3. Alignment: which sensor axis is which body axis.
 *
 *    One matrix per device, row-major, body X row first, applied as
 *    v_body = M * v_sensor. All three are the identity today, and that is a
 *    statement about the mounting and not about the parts: the magnetometer is
 *    on its own breakout board on flying leads, so its axes are parallel to the
 *    IMU's only by luck, and both modules move relative to the board. The
 *    matrices exist so that when the two parts are on one board in a known
 *    orientation the fix is one line here rather than a sign error hunted for
 *    through the filter.
 *
 *    ALIGN_MAG is the identity and is known to be WRONG. On the 2026-09-29
 *    static run the angle between the field and gravity -- each read in its own
 *    part's frame, so with no attitude anywhere in the arithmetic -- came out
 *    64.7 degrees, holding to a few tenths across the run. The dip puts the
 *    field 90 degrees plus the dip away from up, roughly 120 degrees here, so
 *    the two breakout boards are tilted at least 55 degrees apart. ALIGN_ACCEL
 *    and ALIGN_GYRO are the identity for a real reason that ALIGN_MAG lacks:
 *    those two axes are one die.
 *
 *    Why the run still looks healthy, and why this is not visible from a static
 *    one: Fus_ReferenceTake rotates the measured field by the power-on attitude,
 *    so the field reference carries whatever rotation sits between the two
 *    boards and cancels it exactly. The model is then self-consistent at that
 *    one attitude and nowhere else. A static run cannot show this error and
 *    cannot be used to clear it; tilting the assembly and watching the heading
 *    is the test.
 *
 *    The sign convention they carry is worth stating, because the whole filter
 *    hangs off it: at rest the accelerometer reads +1 g along whichever of its
 *    axes points up. A part that reads the other way is not a different
 *    convention to be handled in the arithmetic -- it is a rotation, so it
 *    belongs in this matrix, and the bench check that catches it is a known
 *    tilt coming out with the wrong sign. */
#define APP_FUSION_ALIGN_ACCEL  {  1.0f, 0.0f, 0.0f, \
                                   0.0f, 1.0f, 0.0f, \
                                   0.0f, 0.0f, 1.0f }
#define APP_FUSION_ALIGN_GYRO   {  1.0f, 0.0f, 0.0f, \
                                   0.0f, 1.0f, 0.0f, \
                                   0.0f, 0.0f, 1.0f }
#define APP_FUSION_ALIGN_MAG    {  1.0f, 0.0f, 0.0f, \
                                   0.0f, 1.0f, 0.0f, \
                                   0.0f, 0.0f, 1.0f }

/* Unit conversion. The filter works in radians, in g and in gauss, because that
 * is the form every statement of these equations takes; the sensor's units
 * exist at the two edges only -- milli-degrees per second on the way in,
 * milli-degrees on the way out. */
#define FUS_DEG2RAD                     (0.01745329252f)
#define FUS_RAD2DEG                     (57.29577951f)
#define FUS_MDPS2RADS                   (1.745329252e-5f)
#define FUS_RADS2MDPS                   (57295.77951f)
#define FUS_MG2G                        (1.0e-3f)
#define FUS_MG2GAUSS                    (1.0e-3f)

/* 4. What the attitude is worth.
 *
 *    HAVE_ATTITUDE is set once the startup reference has been taken, and means
 *    the quaternion and the angles beside it are a measurement rather than a
 *    default. HAVE_MAG means the field reference was taken alongside it, so the
 *    magnetometer rows have something to compare against; without it they are
 *    silently dropped and yaw is unobserved again. Both bits are cleared or set
 *    rather than accumulated -- they describe the state now.
 *
 *    HAVE_MAG is not the same claim as "the part is answering right now". The
 *    rows go into the correction only on a tick the magnetometer actually
 *    produced a sample on, and the flag says so. The two parts are polled on one
 *    tick but they do not deliver on one tick: over the 2026-09-29 static run
 *    the console counters read m=341 against p=487, so a finished measurement
 *    was there on about 70% of the ticks and the magnetometer corrected at
 *    roughly two thirds of the accelerometer's rate. That is the right
 *    behaviour rather than a gap to close: a row built from a reading the part
 *    is still holding is a row that claims the body has not moved since the
 *    reading was taken. */
#define APP_FUSION_HAVE_ATTITUDE        (0x01u)     /* the reference is taken  */
#define APP_FUSION_HAVE_MAG             (0x02u)     /* the field reference too */

/* 5. The error state, which is what the covariance is about.
 *
 *    Seven numbers are the state -- four for the quaternion and three for the
 *    bias -- but the filter runs on six, because the four are not independent: a
 *    quaternion names a rotation, and a rotation has three degrees of freedom.
 *    So the covariance is 6x6, and what it describes is a small rotation away
 *    from the nominal quaternion rather than the quaternion itself. That is what
 *    makes a quaternion tractable in a Kalman filter at all: the error is a
 *    plain three-vector, so it can be scaled and averaged and given a
 *    covariance, while the nominal part stays a unit quaternion because nothing
 *    ever adds to it directly. It is normalised after every operation, and the
 *    error is what moves it.
 *
 *    The error is defined in the BODY frame -- q_true = q (x) dq, dq being the
 *    small rotation dtheta/2 -- and that choice is load-bearing. In the body
 *    frame the Jacobian of a vector observation is just the skew matrix of the
 *    vector the state predicts, with no frame rotation in between; in the nav
 *    frame it would carry a rotation of the observation on every row. The
 *    consequence worth knowing at the call site is the sign: the correction is
 *    injected with Fus_RotQuat(), right-multiplied into the quaternion, and an
 *    update applied the other way round would rotate the error into the wrong
 *    frame and walk the attitude away instead of towards the measurement.
 *
 *    The error state is [dtheta; dbias]: the first three are that small body
 *    rotation in radians, the last three are the bias error in radians per
 *    second. The order matters, because it is the order the 6x6 matrices and the
 *    rows of the observation Jacobian are laid out in. */

/**
 * @brief  The latest attitude, for any consumer that is not the console.
 *
 * @note   The quaternion is the state and the Euler angles are derived from it,
 *         so the two can never disagree. The angles are integers in
 *         milli-degrees for the same reason the sensor sample is: this module
 *         keeps its units in the member names and never depends on the
 *         console's idea of a float.
 *
 * @note   Yaw is measured from the heading at power-on, not from north -- see
 *         the frame note at the top of this file.
 *
 * @note   Roll and pitch are the part of the attitude the accelerometer can
 *         correct. Yaw is the part it cannot: the accelerometer's rows have the
 *         predicted direction along their null space, so they say nothing about
 *         a rotation about the vertical, and what keeps yaw from walking is the
 *         magnetometer's rows. When HAVE_MAG is clear -- no field reference was
 *         ever taken -- yaw is a free integrator with the residual bias driving
 *         it, and it will run. That is the state to read the flag in.
 *
 * @note   ResidualMilli and ResidualMagMilli are the lengths of the last
 *         residual between each measurement and the state's prediction of it,
 *         scaled by 1000. The measurements are unit vectors, so each reads as
 *         the angle between where that sensor says its reference direction is
 *         and where the state says it should be -- milli-radians, for a small
 *         disagreement. They are the numbers that say whether the observations
 *         still agree with the state: a residual that climbs is a filter being
 *         dragged, and one that sits at the level its own R predicts is a filter
 *         that has converged. Each is measured against its own R and they are
 *         not comparable -- see the noise note in the header.
 *
 * @note   ResidualMagMilli holds its last value on a tick that carried no
 *         magnetometer row, so it is the last magnetometer correction rather
 *         than the current one. At one console line per five seconds, and with
 *         the two parts at the same nominal rate, that is the same thing in
 *         practice.
 */
typedef struct
{
    float    Qw, Qx, Qy, Qz;        /* body to nav, normalised                 */
    int32_t  RollMdeg;              /* about body X, from the quaternion       */
    int32_t  PitchMdeg;             /* about body Y                            */
    int32_t  YawMdeg;               /* about body Z, zero at power-on          */
    int32_t  AccelMagMg;            /* |a|, milli-g, 1000 at rest              */
    int32_t  MagMgaussMag;          /* |m|, milli-gauss                        */
    int32_t  ResidualMilli;         /* last accel correction, x1000            */
    int32_t  ResidualMagMilli;      /* last mag correction, x1000              */
    uint32_t TickMs;                /* TimeBase the attitude belongs to        */
    uint8_t  Flags;                 /* APP_FUSION_HAVE_*, current state        */
} App_Fusion_Attitude_t;

/**
 * @name    App_Fusion_Init
 * @brief   Reset the filter and open the startup window.
 * @param   None
 * @retval  None
 * @note    Called from Mcal_InitSum(), before the main loop; App_Sensor_Init()
 *          must already have run, because the window is filled from the samples
 *          the sensor module is taking.
 * @note    It does not block. The reference is collected by App_Fusion_Update()
 *          from those same samples, so it is measured under exactly the
 *          conditions the filter will run in -- same slot, same jitter, same
 *          interruptions -- rather than in a quiet loop that exists nowhere
 *          else.
 */
void App_Fusion_Init(void);

/**
 * @name    App_Fusion_Update
 * @brief   Consume one sensor sample: fill the startup window until the
 *          reference is taken, then predict on the gyro and correct on whatever
 *          else arrived.
 * @param   None
 * @retval  None
 * @note    Called from the sample tick, after the sensor module has read the
 *          parts, so the sample it sees is the one that tick produced. A tick
 *          the sensor module filled nothing on is skipped by timestamp, which
 *          is also where the integration step comes from: the gap between two
 *          samples this function actually consumed, clamped to
 *          APP_FUSION_DT_MAX.
 * @note    A tick with no fresh gyro gets no prediction at all, state and
 *          covariance together, and the next tick's longer step covers the gap.
 *          Predicting on a repeated gyro reading would integrate a rate that is
 *          known to be stale, and the measured step is what makes skipping the
 *          honest choice rather than a lost tick.
 * @note    It prints nothing. Everything this module has to say goes out
 *          through App_Fusion_Log(), which the main loop calls: the console
 *          waits on the transmitter per character, and this function runs where
 *          the radio and the network are already waiting.
 */
void App_Fusion_Update(void);

/**
 * @name    App_Fusion_GetAttitude
 * @brief   The latest attitude, for any consumer that is not the console.
 * @param   None
 * @retval  Pointer to the module's one attitude buffer. Never NULL.
 * @note    Check Flags before believing it: HAVE_ATTITUDE stays clear until the
 *          startup reference has been taken, and a cleared bit means the
 *          quaternion is still the identity.
 * @note    The buffer is written in place from the sampling tick's interrupt,
 *          so a consumer in the main loop can be reading it while it changes.
 *          Copy what is needed with interrupts masked, or take it from inside
 *          the tick. The console reads it live and accepts the consequence: the
 *          worst a live read can do is mix two ticks into one line, and that is
 *          only visible while the board is turning.
 */
const App_Fusion_Attitude_t *App_Fusion_GetAttitude(void);

/**
 * @name    App_Fusion_GetGyroBias
 * @brief   The gyro rate bias the filter currently believes.
 * @param   pBiasRads   Receives the bias: three floats, body frame, radians per
 *                      second.
 * @retval  None
 * @note    The predict step subtracts this from the gyro, so it is the startup
 *          window's mean at first and a state estimate afterwards: every
 *          correction moves it. The console converts it to milli-degrees per
 *          second itself, which keeps the module's units in radians.
 * @note    The filter keeps this to itself; it is exposed because the console
 *          line in App_FusionLog_Handle.c prints it, and it is read there
 *          through this call rather than out of the module's state.
 */
void App_Fusion_GetGyroBias(float *pBiasRads);

/**
 * @name    App_Fusion_GetMagRefN
 * @brief   The field reference the startup attitude implies.
 * @param   pRefN   Receives the reference: three floats, nav frame, unit length.
 * @retval  None
 * @note    Zero until the startup reference is taken, and meaningful only when
 *          APP_FUSION_HAVE_MAG is set in App_Fusion_GetAttitude()->Flags: it is
 *          the measured field rotated into the nav frame by the startup
 *          attitude, and the magnetometer's rows compare against it. Written
 *          once and never again, so a late read is the same vector.
 * @note    The filter keeps this to itself; it is exposed because the console
 *          line in App_FusionLog_Handle.c prints it.
 */
void App_Fusion_GetMagRefN(float *pRefN);

/**
 * @name    App_Fusion_GetGyroCount
 * @brief   How many gyro samples the startup window has taken.
 * @param   None
 * @retval  The count, in samples, of the window that produces the reference.
 * @note    The filter keeps this to itself; it is exposed because the console's
 *          pending line in App_FusionLog_Handle.c reports it against
 *          APP_FUSION_BIAS_SAMPLES. It holds its last value once the reference
 *          has been taken, so it is a progress figure only while
 *          APP_FUSION_HAVE_ATTITUDE is clear.
 */
uint32_t App_Fusion_GetGyroCount(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FUSION_HANDLE_H */
