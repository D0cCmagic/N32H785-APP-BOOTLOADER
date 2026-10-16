/**
 * @file    App_VideoTx_Handle.c
 * @brief   Video send service: the card payload paced onto the wireless link.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.0.0
 *
 * @note    Split out of App_R900PNR_Handle.c, which had grown to carry the
 *          radio, this send path and the video receive path at once. What is
 *          left here is the send half: it stages the card payload, builds the
 *          wire frames and paces them with a token bucket.
 * @note    It knows the link only through what the radio hands it -- the peer
 *          address once the radio has learned it, and whether the module has
 *          associated. Nothing here reads the radio's state directly, so the
 *          two modules can be read and changed one at a time.
 */

#include "App_VideoTx_Handle.h"
#include "App_VideoFile_Handle.h"
#include "App_Crc32.h"
#include "hgic_raw.h"
#include "log.h"
#include "main.h"                   /* provides sys_now() */
#include <string.h>

/* Module-private statics */

/* Broadcast until the radio learns who is on the other end */
static uint8_t  Peer_Mac[6]    = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};

static uint8_t  Tx_Started     = 0;
static uint32_t Tx_Total_Bytes = 0;
static uint32_t Tx_Total_Pkts  = 0;
static uint32_t Tx_Fail_Cnt    = 0;

/* Video streaming state */
static uint8_t  Vid_Tx_Frame[VID_FRAME_LEN];
static uint8_t  Vid_Stage[VID_SD_CHUNK_BYTES + VID_FRAME_PAYLOAD_LEN];

static uint32_t Vid_Stage_Len   = 0u;   /* valid bytes at the front of Vid_Stage */
static uint32_t Vid_Stage_Pos   = 0u;   /* next byte to hand to a frame          */
static uint32_t Vid_File_Offset = 0u;   /* next byte to pull off the card        */
static uint32_t Vid_Seq         = 0u;   /* frame counter, goes out in the header */
static uint32_t Vid_Budget      = 0u;   /* token bucket, counted in frame bytes  */
static uint32_t Vid_Last_Tick   = 0u;
static uint8_t  Vid_Ready       = 0u;   /* 0 until the header has been read      */
static uint32_t Vid_Open_Tries  = 0u;   /* header read attempts made so far      */
static uint32_t Vid_Open_Tick   = 0u;   /* when the last attempt was made        */
static uint32_t Vid_Read_Fails  = 0u;   /* consecutive failed card reads         */
static uint32_t Vid_Read_Log_Tick = 0u; /* when a read failure was last reported */
#if (VID_TX_TRACE != 0u)
static uint32_t Tx_Trace_Cnt    = 0u;
#endif

/**
 * @name    App_VideoTx_SetPeer
 * @brief   Tell the send path which address to put on the wire.
 * @param   Mac: the peer's 6 byte address.
 * @retval  None
 * @note    Called by the radio when it works the address out of a frame that
 *          came from the other end. Until then the frames carry the broadcast
 *          address, which is how the first of them reach the far side.
 */
void App_VideoTx_SetPeer(const uint8_t *Mac)
{
    if (Mac == NULL)
    {
        return;
    }

    memcpy(Peer_Mac, Mac, 6u);
}

/**
 * @name    App_VideoStreamOpen
 * @brief   Load the card header and reset every streaming position.
 * @param   None
 * @retval  VIDEO_OK, or a negative VideoFile_Status_t.
 */
static int App_VideoStreamOpen(void)
{
    int rc = App_VideoFile_Open();

    Vid_Stage_Len   = 0u;
    Vid_Stage_Pos   = 0u;
    Vid_File_Offset = 0u;
    Vid_Seq         = 0u;
    Vid_Budget      = 0u;
    Vid_Last_Tick   = sys_now();
    Vid_Ready       = (rc == VIDEO_OK) ? 1u : 0u;

    return rc;
}

/**
 * @name    VidStream_Refill
 * @brief   Top the staging buffer up so at least one whole frame can be built.
 * @param   None
 * @retval  None
 * @note    The clip loops: once every byte has been handed out the read
 *          position goes back to the start.
 */
static void VidStream_Refill(void)
{
    uint32_t held;
    uint32_t got = 0u;

    held = Vid_Stage_Len - Vid_Stage_Pos;
    if (held >= VID_FRAME_PAYLOAD_LEN)
    {
        return;
    }

    /* Slide the leftover to the front so the read has room */
    if ((held > 0u) && (Vid_Stage_Pos > 0u))
    {
        memmove(Vid_Stage, &Vid_Stage[Vid_Stage_Pos], held);
    }
    Vid_Stage_Pos = 0u;
    Vid_Stage_Len = held;

    if (Vid_File_Offset >= App_VideoFile_GetStreamBytes())
    {
        Vid_File_Offset = 0u;
    }

    if (App_VideoFile_ReadBytes(Vid_File_Offset, &Vid_Stage[Vid_Stage_Len],
                                VID_SD_CHUNK_BYTES, &got) == VIDEO_OK)
    {
        Vid_Stage_Len   += got;
        Vid_File_Offset += got;
        Vid_Read_Fails   = 0u;
    }
    else
    {
        /* The next call asks for the same block again, so the retry is implicit
         * and the offset is left where it is on purpose. Counted, and printed at
         * most once per VID_READ_FAIL_LOG_MS, so that a card that has come loose
         * stops the stream with a reason on the console instead of in silence. */
        Vid_Read_Fails++;

        if ((Vid_Read_Fails == 1u) ||
            ((sys_now() - Vid_Read_Log_Tick) >= VID_READ_FAIL_LOG_MS))
        {
            Vid_Read_Log_Tick = sys_now();

            log_info("[TX] SD read failed (%d in a row), stalled at offset %d\r\n",
                     (int)Vid_Read_Fails, (int)Vid_File_Offset);
        }
    }
}

/**
 * @name    VidStream_SendFrame
 * @brief   Build one video frame from staged bytes and push it onto the link.
 * @param   None
 * @retval  Number of bytes sent, or a negative value on error.
 * @note    The header is written byte by byte: the frame buffer must never be
 *          cast to a struct, alignment and padding are not guaranteed.
 */
static int VidStream_SendFrame(void)
{
    uint32_t crc;
    int      ret;

    Vid_Tx_Frame[0] = (uint8_t)(VID_FRAME_MAGIC & 0xFFu);
    Vid_Tx_Frame[1] = (uint8_t)((VID_FRAME_MAGIC >> 8) & 0xFFu);

    Vid_Tx_Frame[2] = (uint8_t)(Vid_Seq);
    Vid_Tx_Frame[3] = (uint8_t)(Vid_Seq >> 8);
    Vid_Tx_Frame[4] = (uint8_t)(Vid_Seq >> 16);
    Vid_Tx_Frame[5] = (uint8_t)(Vid_Seq >> 24);

    /* Flags stay clear, the clip loops so there is no last frame */
    Vid_Tx_Frame[6] = 0u;
    Vid_Tx_Frame[7] = 0u;

    memcpy(&Vid_Tx_Frame[VID_FRAME_HDR_LEN], &Vid_Stage[Vid_Stage_Pos],
           VID_FRAME_PAYLOAD_LEN);
    Vid_Stage_Pos += VID_FRAME_PAYLOAD_LEN;

    crc = App_Crc32(&Vid_Tx_Frame[VID_FRAME_HDR_LEN],
                              VID_FRAME_PAYLOAD_LEN);
    Vid_Tx_Frame[8]  = (uint8_t)(crc);
    Vid_Tx_Frame[9]  = (uint8_t)(crc >> 8);
    Vid_Tx_Frame[10] = (uint8_t)(crc >> 16);
    Vid_Tx_Frame[11] = (uint8_t)(crc >> 24);

#if (VID_CRC_SELF_TEST != 0u)
    Vid_Tx_Frame[VID_FRAME_HDR_LEN] ^= 0x5Au;
#endif

#if (VID_TX_TRACE != 0u)
    if (Tx_Trace_Cnt < VID_TX_TRACE_FRAMES)
    {
        Tx_Trace_Cnt++;
        log_info("[TX TRC] seq=%d crc=%08X f8=%02X%02X%02X%02X tail=%02X%02X%02X%02X\r\n",
                 (int)Vid_Seq, (unsigned int)crc,
                 Vid_Tx_Frame[8], Vid_Tx_Frame[9],
                 Vid_Tx_Frame[10], Vid_Tx_Frame[11],
                 Vid_Tx_Frame[VID_FRAME_LEN - 4u], Vid_Tx_Frame[VID_FRAME_LEN - 3u],
                 Vid_Tx_Frame[VID_FRAME_LEN - 2u], Vid_Tx_Frame[VID_FRAME_LEN - 1u]);
    }
#endif

    ret = hgic_raw_send(Peer_Mac, Vid_Tx_Frame, VID_FRAME_LEN);

    if (ret > 0)
    {
        Vid_Seq++;
    }

    return ret;
}

/**
 * @name    App_VideoStreamSend
 * @brief   Pace the card payload onto the wireless link and log the send rate.
 * @param   Associated: non-zero once the module reports it has associated.
 * @param   AssocAgeMs: milliseconds since association was asked for.
 * @retval  None
 * @note    A token bucket holds the rate at VID_TARGET_KBPS. Sending flat out
 *          would overrun the measured link and lose frames.
 * @note    The link state arrives as arguments because it belongs to the radio
 *          module: this one only decides what to do about it.
 */
void App_VideoStreamSend(uint8_t Associated, uint32_t AssocAgeMs)
{
    uint32_t now;
    uint32_t elapsed;

    if (!Tx_Started)
    {
        if (Associated)
        {
            log_info("[TX] WiFi connected\r\n");
        }
        else if (AssocAgeMs < APP_ASSOC_TIMEOUT_MS)
        {
            return;
        }
        else
        {
            log_info("[TX] assoc timeout, start sending anyway\r\n");

            hgic_raw_get_disassoc_reason();
        }

        Tx_Started = 1;
    }

    /* The header is read here, apart from the wait above so that the wait is not
     * repeated with it. It is retried, because one attempt used to be all it
     * got: a card that is slow to answer after power-up meant no video for the
     * rest of the run, and the console said only that there was no usable
     * payload -- which is also what a card with no clip on it says. */
    if (Vid_Ready == 0u)
    {
        if (Vid_Open_Tries >= VID_OPEN_TRIES)
        {
            return;
        }
        if ((Vid_Open_Tries > 0u) && ((sys_now() - Vid_Open_Tick) < VID_OPEN_RETRY_MS))
        {
            return;
        }

        Vid_Open_Tries++;
        Vid_Open_Tick = sys_now();

        if (App_VideoStreamOpen() == VIDEO_OK)
        {
            if (Vid_Open_Tries > 1u)
            {
                log_info("[TX] streaming the card payload, header took %d tries\r\n",
                         (int)Vid_Open_Tries);
            }
            else
            {
                log_info("[TX] streaming the card payload\r\n");
            }
        }
        else
        {
            if (Vid_Open_Tries >= VID_OPEN_TRIES)
            {
                log_info("[TX] no usable payload on the card after %d tries,"
                         " not sending\r\n", (int)Vid_Open_Tries);
            }

            return;
        }
    }

    now     = sys_now();
    elapsed = now - Vid_Last_Tick;
    if (elapsed == 0u)
    {
        return;
    }
    Vid_Last_Tick = now;

    Vid_Budget += elapsed * VID_BYTES_PER_MS;
    if (Vid_Budget > VID_BUDGET_CAP)
    {
        Vid_Budget = VID_BUDGET_CAP;
    }

    while (Vid_Budget >= VID_FRAME_LEN)
    {
        VidStream_Refill();
        if ((Vid_Stage_Len - Vid_Stage_Pos) < VID_FRAME_PAYLOAD_LEN)
        {
            break;      /* the card fell behind, pick it up on the next call */
        }

        if (VidStream_SendFrame() > 0)
        {
            Tx_Total_Bytes += VID_FRAME_LEN;
            Tx_Total_Pkts++;
        }
        else
        {
            Tx_Fail_Cnt++;
        }

        Vid_Budget -= VID_FRAME_LEN;
    }
}

/**
 * @name    App_VideoTxGetStats
 * @brief   Read the send counters and clear them for the next window.
 * @param   pStats: destination for the counters.
 * @retval  None
 */
void App_VideoTxGetStats(App_VideoTxStats_t *pStats)
{
    if (pStats == NULL)
    {
        return;
    }

    pStats->Pkts  = Tx_Total_Pkts;
    pStats->Bytes = Tx_Total_Bytes;
    pStats->Fail  = Tx_Fail_Cnt;
    pStats->Seq   = Vid_Seq;

    Tx_Total_Pkts  = 0;
    Tx_Total_Bytes = 0;
    Tx_Fail_Cnt    = 0;
}
