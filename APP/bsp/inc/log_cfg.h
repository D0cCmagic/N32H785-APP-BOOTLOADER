#ifndef __LOG_CFG_H__
#define __LOG_CFG_H__

/**
 * @file    log_cfg.h
 * @brief   What the console prints on its own, with nobody asking.
 *
 * @note    The console and the protocol share one port and one transmit path,
 *          and the transmit path is synchronous: a line waits for the
 *          transmitter byte by byte. So a periodic line is not just noise on the
 *          screen -- it interleaves with whatever frame is going out at that
 *          moment, and a frame with a log line spliced into it fails its
 *          checksum at the far end. The failure looks like a protocol fault and
 *          is painful to trace, because both halves look healthy on their own.
 *
 * @note    So the unsolicited output is behind this switch rather than printed
 *          unconditionally. While the link is being characterised, and while
 *          checksum failures are the thing being watched, it stays off. Turn it
 *          back on when the numbers themselves are what is being looked at.
 *
 * @note    This is a stopgap, not the fix. The fix is for the transmit path to
 *          queue a frame ahead of a log line, or for the log to be dropped while
 *          a frame is in flight; neither exists yet. This switch only stops the
 *          collision from happening during bring-up.
 *
 * @note    Event-driven output is deliberately NOT behind this switch: the
 *          fault lines, the sensor bus scan and the calibration report all say
 *          something that happened, print at most once, and are the reason the
 *          console exists. Only the lines that repeat on a timer are gated.
 */

/* 1 -- the periodic sensor/status lines are printed.
 * 0 -- they are compiled out. */
#ifndef LOG_PERIODIC_ENABLE
#define LOG_PERIODIC_ENABLE     (0)
#endif

#endif /* __LOG_CFG_H__ */
