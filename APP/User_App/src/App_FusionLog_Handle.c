/**
 * @file    App_FusionLog_Handle.c
 * @brief   The console side of the attitude filter: the one-slot queue of
 *          messages the step raises, and the lines printed for them.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.0.0
 *
 * @note    Split out of App_Fusion_Handle.c with no change of behaviour: every
 *          line below was moved, not rewritten. What stayed behind is the
 *          filter -- and with it the decision that a line is owed, which is why
 *          the messages arrive here through App_FusionLog_Post() rather than
 *          being set in place.
 *
 *          See App_FusionLog_Handle.h for why the split falls where it does.
 *          The values the lines print are read at print time through the
 *          filter's read-only accessors, exactly as they were read when both
 *          halves were one file; only what will not keep -- a window's length
 *          and its sample count -- is carried in the queue.
 *
 *          Nothing in this file's tick-side entry point prints: Post() is a
 *          store, and every log_info() below runs from App_Fusion_Log(), which
 *          the scheduler calls from its 10 ms slot.
 */

#include "App_FusionLog_Handle.h"
#include "App_Fusion_Handle.h"
#include "log.h"
#include "log_cfg.h"
#include "main.h"                   /* provides extern volatile uint32_t TimeBase */

/* One slot is enough for the three messages above: each is a once-per-run
 * diagnostic, and the main loop drains the slot within a tick of it being
 * raised. What the lines need that will not keep -- a window's length, a
 * sample count -- is carried in the two numbers. */
static volatile uint8_t Fus_PendingMsg = FUS_MSG_NONE;
static uint32_t Fus_PendingMs;
static uint32_t Fus_PendingCount;

static uint32_t Fus_LastLogTick;

static void Fus_LogPending(void);

/**
 * @name    App_FusionLog_Init
 * @brief   Start the console's slow gate from now.
 * @param   None
 * @retval  None
 * @note    Called from App_Fusion_Init(), which is where this counter was wound
 *          when it lived in the filter, so the gate is measured from the
 *          instant the module is initialised rather than from its first call.
 */
void App_FusionLog_Init(void)
{
    Fus_LastLogTick = TimeBase;
}

/**
 * @name    App_FusionLog_Post
 * @brief   Raise a one-off console message for the main loop to print.
 * @param   Msg     One of the FUS_MSG_* ids.
 * @param   Ms      The window's length in milliseconds, printed by the
 *                  reference line; meaningless for the other messages.
 * @param   Count   The window's sample count, printed by the reference line and
 *                  by the stall line.
 * @retval  None
 * @note    This runs in the sample tick, so it stores and returns: the waiting
 *          on the transmitter happens in App_Fusion_Log(), on the main loop.
 * @note    The message is written last, after the numbers it describes. The
 *          message is the only one of the three that is volatile, because it is
 *          the flag the main loop tests.
 */
void App_FusionLog_Post(uint8_t Msg, uint32_t Ms, uint32_t Count)
{
    Fus_PendingMs    = Ms;
    Fus_PendingCount = Count;
    Fus_PendingMsg   = Msg;
}

/**
 * @name    Fus_LogPending
 * @brief   Report a startup window that has still not filled.
 * @param   None
 * @retval  None
 * @note    The attitude used to be reported from here once per period. It is not
 *          any more: the console carries the transfer rate and the raw axes, and
 *          anything that wants the attitude is a consumer of
 *          App_Fusion_GetAttitude(), which is what the public interface is for.
 *          What is left is the one condition worth breaking the quiet for -- a
 *          window still waiting for samples, which on a working board is over in
 *          about a second and on a board whose IMU never came up never ends.
 */
static void Fus_LogPending(void)
{
#if (LOG_PERIODIC_ENABLE == 0)
    /* Periodic while the filter waits for its reference -- off by default, see
     * log_cfg.h. */
    (void)0;
#else
    if ((App_Fusion_GetAttitude()->Flags & APP_FUSION_HAVE_ATTITUDE) != 0u)
    {
        return;
    }

    log_info("[FUS] reference pending: %d/%d gyro samples\r\n",
             (int)App_Fusion_GetGyroCount(), (int)APP_FUSION_BIAS_SAMPLES);
#endif
}

/**
 * @name    App_Fusion_Log
 * @brief   Print anything the filter is holding for the console.
 * @param   None
 * @retval  None
 * @note    Called from the scheduler's 10 ms slot, which is the one place in
 *          this module where waiting on the transmitter is allowed. The step
 *          itself prints nothing, so that it can run in the tick interrupt.
 * @note    The two jobs are independent: any message the step raised, and then
 *          the slow gate that watches for a startup window which never fills. A
 *          run with nothing pending and its reference already taken prints
 *          nothing here at all.
 */
void App_Fusion_Log(void)
{
    uint32_t Now = TimeBase;
    uint8_t  Kind;
    const App_Fusion_Attitude_t *pAtt;
    float Bias[3];
    float RefN[3];

    /* Drained in a loop rather than one per call: the messages are rare but
     * they are not exclusive, and a stall can be raised in the same window as
     * a Cholesky failure. */
    while (Fus_PendingMsg != FUS_MSG_NONE)
    {
        Kind           = Fus_PendingMsg;
        Fus_PendingMsg = FUS_MSG_NONE;

        if (Kind == FUS_MSG_REFERENCE)
        {
            /* Read here, at the moment the line is printed, rather than carried
             * from when the message was raised: the filter has published since,
             * and this is the state the console is looking at. */
            pAtt = App_Fusion_GetAttitude();
            App_Fusion_GetGyroBias(Bias);
            App_Fusion_GetMagRefN(RefN);

            log_info("[FUS] reference: gyro bias %d %d %d mdps from %d samples in %d ms\r\n",
                     (int)(Bias[0] * FUS_RADS2MDPS),
                     (int)(Bias[1] * FUS_RADS2MDPS),
                     (int)(Bias[2] * FUS_RADS2MDPS),
                     (int)Fus_PendingCount, (int)Fus_PendingMs);
            log_info("[FUS] reference: |a|=%d mg |m|=%d mG, roll %d pitch %d yaw %d mdeg,"
                     " field %d %d %d /1000\r\n",
                     (int)pAtt->AccelMagMg, (int)pAtt->MagMgaussMag,
                     (int)pAtt->RollMdeg, (int)pAtt->PitchMdeg,
                     (int)pAtt->YawMdeg,
                     (int)(RefN[0] * 1000.0f), (int)(RefN[1] * 1000.0f),
                     (int)(RefN[2] * 1000.0f));
        }
        else if (Kind == FUS_MSG_STALL)
        {
            log_info("[FUS] no sample for %d ms, reference still waiting"
                     " (%d/%d)\r\n", (int)APP_FUSION_BIAS_STALL_MS,
                     (int)Fus_PendingCount, (int)APP_FUSION_BIAS_SAMPLES);
        }
        else if (Kind == FUS_MSG_CHOLFAIL)
        {
            log_info("[FUS] residual covariance is not positive definite,"
                     " correction skipped\r\n");
        }
    }

    if ((Now - Fus_LastLogTick) < APP_FUSION_LOG_PERIOD_MS)
    {
        return;
    }
    Fus_LastLogTick = Now;
    Fus_LogPending();
}
