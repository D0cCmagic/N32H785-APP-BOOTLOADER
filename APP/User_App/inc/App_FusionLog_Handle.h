/**
 * @file    App_FusionLog_Handle.h
 * @brief   The console side of the attitude filter: the one-off messages the
 *          step raises, and the lines the main loop prints for them.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.0.0
 *
 * @note    Split out of App_Fusion_Handle.h with no change of behaviour. What
 *          moved is everything on the console side of that module: the one-slot
 *          queue of pending messages, the ids those messages carry, the slow
 *          gate that reports a startup window which never fills, and the
 *          printing of the three lines -- the startup reference, a stalled
 *          window and a failed Cholesky factorisation. What stayed behind is
 *          the filter: App_Fusion_Update() and everything it calls, which is also
 *          where the decision that a line is owed is taken.
 *
 *          The two halves meet at App_FusionLog_Post(). The filter raises a
 *          message and hands over the two numbers that will not keep -- a
 *          window's length and its sample count -- and nothing here reads or
 *          writes the filter's state: what a line prints is read at print time
 *          through the filter's read-only accessors, App_Fusion_GetAttitude(),
 *          App_Fusion_GetGyroBias() and App_Fusion_GetMagRefN(), which is what
 *          the arithmetic did when both halves were one file.
 *
 *          The hard rule this project keeps is part of why the split falls
 *          where it does: every printf is on the main loop path and never in an
 *          interrupt. App_FusionLog_Post() is called from the sample tick and
 *          is a store and nothing else; App_Fusion_Log() is called from the
 *          scheduler's 10 ms slot, which is the one context here that is
 *          allowed to wait on the transmitter.
 */

#ifndef _APP_FUSIONLOG_HANDLE_H_
#define _APP_FUSIONLOG_HANDLE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Which one-off console line the step is holding for the main loop to print.
 * The step must not print for itself: log_info() polls the transmitter one
 * character at a time, and the step is meant to run in the tick interrupt,
 * where that wait is charged to the radio and the network. */
#define FUS_MSG_NONE                    (0u)
#define FUS_MSG_REFERENCE               (1u)
#define FUS_MSG_STALL                   (2u)
#define FUS_MSG_CHOLFAIL                (3u)

/**
 * @name    App_FusionLog_Init
 * @brief   Start the console's slow gate from now.
 * @param   None
 * @retval  None
 * @note    Called at the end of App_Fusion_Init(), which is where the counter
 *          behind that gate was wound when it lived in the filter. The first
 *          periodic line therefore still arrives a full
 *          APP_FUSION_LOG_PERIOD_MS after the module is initialised, rather
 *          than at the first call.
 */
void App_FusionLog_Init(void);

/**
 * @name    App_FusionLog_Post
 * @brief   Raise a one-off console message for the main loop to print.
 * @param   Msg     One of the FUS_MSG_* ids.
 * @param   Ms      The window's length in milliseconds, which only the
 *                  reference line prints; 0 when the message carries none.
 * @param   Count   The window's sample count, which the reference line and the
 *                  stall line print; 0 when the message carries none.
 * @retval  None
 * @note    Called from the filter's step, which runs in the sample tick, so
 *          this must not print -- and does not: it writes the two numbers and
 *          then the message, and returns. log_info() is called by
 *          App_Fusion_Log() from the main loop.
 * @note    The message is written last, after the numbers it is about, which is
 *          the order the three assignments were in when they lived in the
 *          filter. Fus_PendingMsg is volatile for the reason it was there:
 *          written from the tick, read from the loop.
 * @note    One slot, and it is not a queue in depth. A second message raised
 *          before the loop drains the first replaces it, which is the
 *          behaviour the filter had with its one queue variable.
 */
void App_FusionLog_Post(uint8_t Msg, uint32_t Ms, uint32_t Count);

/**
 * @name    App_Fusion_Log
 * @brief   Print anything the filter is holding for the console.
 * @param   None
 * @retval  None
 * @note    Called from the scheduler's 10 ms slot, where a blocking write costs
 *          nothing that is waiting on it. It prints the messages the step raised
 *          -- the startup reference, a stalled window, a failed Cholesky
 *          factorisation -- and, on its own slow gate, a window that still has
 *          not filled. It prints no attitude: the console carries the transfer
 *          rate and the raw sensor axes, and anything that wants the attitude is
 *          a consumer of App_Fusion_GetAttitude().
 */
void App_Fusion_Log(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_FUSIONLOG_HANDLE_H_ */
