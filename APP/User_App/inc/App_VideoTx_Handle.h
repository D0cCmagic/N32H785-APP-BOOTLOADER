#ifndef _APP_VIDEOTX_HANDLE_H_
#define _APP_VIDEOTX_HANDLE_H_

/**
 * @file    App_VideoTx_Handle.h
 * @brief   Video send service: the card payload paced onto the wireless link.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.0.0
 *
 * @note    Split out of App_R900PNR_Handle.h, which used to carry the radio,
 *          the video send path and the video receive path in one module.
 */

#include "stdint.h"

/* Video frame format on the wireless link.
 *
 *   offset 0   magic     uint16   0x4E56, marks a video frame
 *   offset 2   seq       uint32   frame counter, +1 per frame
 *   offset 6   flags     uint8    bit0 set on the last frame of the stream
 *   offset 7   reserved  uint8
 *   offset 8   crc32     uint32   CRC32 of this frame payload
 *   offset 12  payload   VID_FRAME_PAYLOAD_LEN bytes
 */
#define VID_FRAME_MAGIC                 (0x4E56u)
#define VID_FRAME_HDR_LEN               (12u)
#define VID_FRAME_LEN                   (1400u)
#define VID_FRAME_PAYLOAD_LEN           (VID_FRAME_LEN - VID_FRAME_HDR_LEN)
#define VID_FRAME_FLAG_LAST             (0x01u)

/* The frame this module builds is exactly what goes on the wire, so the two
 * sizes have to agree. The check used to live in the radio's header, next to a
 * payload macro that only this module ever used. */
#define APP_TX_PAYLOAD_LEN              (1400u)

#if (VID_FRAME_LEN != APP_TX_PAYLOAD_LEN)
#error "The STA must send exactly the wire frame size"
#endif

#define VID_CRC_SELF_TEST               (0u)
#define VID_TX_TRACE                    (0u)
#define VID_TX_TRACE_FRAMES             (8u)

/* Target send rate for the video stream, in kbit/s. */
#define VID_TARGET_KBPS                 (2000u)

/* Payload bytes allowed per millisecond, the token bucket increment. */
#define VID_BYTES_PER_MS                ((VID_TARGET_KBPS * 1000u) / 8u / 1000u)

#define VID_BUDGET_CAP                  (VID_FRAME_LEN * 12u)

/* One SD read per refill, in bytes. Must be a multiple of 512. */
#define VID_SD_CHUNK_BYTES              (8192u)

/* How long the send path waits for the module to associate before it starts
 * sending anyway. The wait belongs with the stream, not with the radio: it is
 * about when this module is willing to begin. */
#define APP_ASSOC_TIMEOUT_MS            (3000u)

/*
 * The card header is read more than once when the first attempt fails, which
 * it does on this board when the card is slow to answer after power-up. The
 * attempts are spaced, because time is what a slow card needs, and the failure
 * is reported once the tries are used up -- a single attempt used to mean no
 * video for the rest of the run with nothing on the console to say why.
 */
#define VID_OPEN_TRIES                  (5u)
#define VID_OPEN_RETRY_MS               (500u)

/*
 * How often a run of failed card reads is reported. The read itself already
 * retries: a failure leaves the offset where it was, so the next call asks for
 * the same block. What was missing is the report -- of a card that has come
 * loose, the only symptom used to be the rate on the statistics line falling to
 * zero, which says nothing about which side of the link stopped.
 */
#define VID_READ_FAIL_LOG_MS            (5000u)

/* Running counters. The radio's statistics line prints them, so they leave this
 * module through a getter rather than through shared globals. */
typedef struct
{
    uint32_t Pkts;      /* frames handed to the radio                          */
    uint32_t Bytes;     /* wire bytes those frames carried                     */
    uint32_t Fail;      /* send calls the radio refused                        */
    uint32_t Seq;       /* next sequence number, so frames built in total      */
} App_VideoTxStats_t;

/**
 * @name    App_VideoTx_SetPeer
 * @brief   Tell the send path which address to put on the wire.
 * @param   Mac: the peer's 6 byte address. Until this is called the frames go
 *          to the broadcast address, which is how the first ones get through.
 * @retval  None
 */
void App_VideoTx_SetPeer(const uint8_t *Mac);

/**
 * @name    App_VideoStreamSend
 * @brief   Pace the card payload onto the wireless link and log the send rate.
 * @param   Associated: non-zero once the module reports it has associated.
 * @param   AssocAgeMs: milliseconds since association was asked for.
 * @retval  None
 */
void App_VideoStreamSend(uint8_t Associated, uint32_t AssocAgeMs);

/**
 * @name    App_VideoTxGetStats
 * @brief   Read the send counters and clear them for the next window.
 * @param   pStats: destination for the counters.
 * @retval  None
 * @note    Reading closes the window, because that is what the statistics line
 *          wants: it prints what accumulated since the last one.
 */
void App_VideoTxGetStats(App_VideoTxStats_t *pStats);

#endif /* _APP_VIDEOTX_HANDLE_H_ */
