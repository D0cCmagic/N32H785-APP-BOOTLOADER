#ifndef __APP_RCCMD_HANDLE_H__
#define __APP_RCCMD_HANDLE_H__

/**
 * @file    App_RcCmd_Handle.h
 * @brief   Command layer: a whole frame in, the answer (if any) out.
 *
 * @note    This layer sits on top of App_Cmd_Handle, which has already found
 *          the frame, checked its tail and verified its checksum. So what
 *          arrives here is always a real frame, and this layer only has to
 *          decide what it means and what to answer. It never looks at bytes,
 *          lengths or checksums.
 *
 * @note    The command byte carries the flags:
 *            bit7 ACK  0 = request or report, 1 = answer
 *            bit6 ERR  0 = fine, 1 = error
 *            bit5..0 CODE
 *
 * @note    The sequence byte, when present, is DATA[0] (the handover doc's
 *          option S1). A reply mirrors the request: it carries the same
 *          sequence byte the request carried, and none when the request had
 *          none. That keeps the doc's example frames valid either way.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Command codes (handover doc section 2.3). */
#define APP_RCCMD_HEARTBEAT      (0x00u)

/* Error code carried in DATA when ERR is set (section 2.5). */
#define APP_RCCMD_ERR_CRC        (0x01u)
#define APP_RCCMD_ERR_LEN        (0x02u)
#define APP_RCCMD_ERR_UNSUPPORT  (0x03u)
#define APP_RCCMD_ERR_PARAM      (0x04u)

/**
 * @name    App_RcCmd_Handle
 * @brief   Act on one frame that App_Cmd_Handle has already validated.
 * @param   Frame: the whole frame, HEAD .. TAIL.
 * @param   FrameLen: its length in bytes.
 * @retval  None
 * @note    Answers are sent from here. A frame that is itself an answer (ACK
 *          set) is not answered -- doing so would talk to itself forever.
 */
void App_RcCmd_Handle(const uint8_t *Frame, uint16_t FrameLen);

/**
 * @name    app_uart_handler
 * @brief   One complete burst from the link: hand it to whoever wants it.
 * @param   None
 * @retval  None
 * @note    Called from the scheduler's 10 ms slot, and it is the only reader of
 *          the driver's burst buffer.
 * @note    It does not move the bytes and does not parse them. It checks
 *          whether a burst is waiting, consumes it by clearing the driver's
 *          flag, and routes the same bytes to each consumer in turn -- the OTA
 *          byte matcher, then the frame layer. Each of those knows its own
 *          format; this function knows only that they both want the bytes.
 * @note    Consuming is what clears the flag: the driver will not touch the
 *          buffer until it is cleared, so the flag being down is the whole of
 *          "this burst is dealt with".
 */
void app_uart_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RCCMD_HANDLE_H__ */
