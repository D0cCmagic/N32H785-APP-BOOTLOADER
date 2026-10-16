#ifndef __APP_VIDEORX_HANDLE_H__
#define __APP_VIDEORX_HANDLE_H__

#include "stdint.h"

/**
 * @name    VideoRxStats_t
 * @brief   Counters for the received video stream.
 * @note    All five are exact, cumulative for the life of the board, and are
 *          the numbers to read when the stream misbehaves at the consumer end.
 *          ReadBytes is what the consumer has taken; TotalBytes is what
 *          arrived; the two differ by whatever is still sitting in the buffer.
 *          A DroppedBytes that keeps growing means the consumer is not calling
 *          App_VideoRxRead often enough - it is not a link problem and no
 *          amount of radio tuning will fix it.
 *
 *          LostFrames and BadCrcFrames are the other two ways a stream goes
 *          wrong and neither is the consumer's doing: a frame counted in
 *          LostFrames never arrived at all, a frame counted in BadCrcFrames
 *          arrived damaged and was refused. Both leave the same kind of hole
 *          in the bytes handed over, and one hole costs as much picture as
 *          the encoder's key frame interval - read these two before blaming
 *          the decoder for what it was fed.
 */
typedef struct
{
    uint32_t TotalBytes;    /**< payload bytes received and made readable */
    uint32_t ReadBytes;     /**< payload bytes the consumer has taken away */
    uint32_t DroppedBytes;  /**< bytes lost because the buffer was full     */
    uint32_t LostFrames;    /**< numbered frames that never turned up      */
    uint32_t BadCrcFrames;  /**< frames refused, their CRC did not match   */
} VideoRxStats_t;

/**
 * @name    App_VideoRxAvailable
 * @brief   How many bytes are readable right now.
 * @param   None
 * @retval  Byte count, always less than the buffer size.
 */
uint32_t App_VideoRxAvailable(void);

/**
 * @name    App_VideoRxRead
 * @brief   Take the oldest received bytes out of the buffer.
 * @param   pBuf: destination, must hold MaxLen bytes.
 * @param   MaxLen: most bytes to take this call.
 * @retval  Bytes actually taken. 0 means nothing is available yet, which is
 *          normal while the sender is starting up and is not an error.
 * @note    The bytes come out in the order they arrived, with the 12 byte
 *          per-frame header already stripped, so they are the H.265 elementary
 *          stream as it sits on the sender's card. Call this from the same
 *          loop that feeds the OTG interface: the sender produces about
 *          500 KB/s and the buffer holds 32 KB, so a consumer that waits more
 *          than roughly 60 ms between calls will start losing data - that loss
 *          shows up in VideoRxStats_t.DroppedBytes, and this function never
 *          blocks to prevent it.
 */
uint32_t App_VideoRxRead(uint8_t *pBuf, uint32_t MaxLen);

/**
 * @name    App_VideoRxGetStats
 * @brief   Read the running counters.
 * @param   pStats: destination for the counters.
 * @retval  None
 */
void App_VideoRxGetStats(VideoRxStats_t *pStats);

#endif
