#include "app_r900pnr_handle.h"
#include "App_Crc32.h"
#include "App_VideoRx_Handle.h"
#if (APP_WIFI_ROLE == APP_ROLE_STA)
#include "App_VideoFile_Handle.h"
#endif
#include <string.h>

/* Module-private statics */
static uint8_t Spi_Rx_Buf[2048u];

#if (APP_WIFI_ROLE == APP_ROLE_STA)
static uint8_t  Wifi_Connected = 0;
static uint8_t  Tx_Started = 0;
static uint32_t Assoc_Start_Tick = 0;
static uint32_t Tx_Total_Bytes = 0;
static uint32_t Tx_Total_Pkts  = 0;
static uint32_t Tx_Fail_Cnt    = 0;
static uint32_t Tx_Rx_Cnt      = 0;
static uint32_t Tx_Last_Print_Tick = 0;
static uint32_t Tx_Air_Cnt     = 0u;
static uint32_t Tx_Air_Probe   = 0u;

static uint8_t  Peer_Mac[6]    = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
static uint8_t  Peer_Mac_Valid = 0u;

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
#if (VID_TX_TRACE != 0u)
static uint32_t Tx_Trace_Cnt    = 0u;
#endif
#endif

#if (APP_WIFI_ROLE == APP_ROLE_AP)
/* Receive ring for the byte stream handed to the consumer. Payload only, the
 * 12 byte frame header is stripped, so draining it yields the H.265 bytes back
 * to back. When full it overwrites the oldest data rather than blocking. */
#define VID_RX_RING_BYTES   (32u * 1024u)
static uint8_t  Rx_Ring[VID_RX_RING_BYTES];
static uint32_t Rx_Ring_Wr      = 0u;
static uint32_t Rx_Ring_Rd      = 0u;
static uint32_t Rx_Ring_Count   = 0u;
static uint32_t Rx_Ring_Written = 0u;
static uint32_t Rx_Ring_Dropped = 0u;
static uint32_t Rx_Ring_Read    = 0u;

static uint32_t Rx_Total_Pkts  = 0;
static uint32_t Rx_Vid_Pkts    = 0;
static uint32_t Rx_Other_Pkts  = 0;
static uint32_t Rx_Vid_Bytes   = 0;
static uint32_t Rx_Lost_Pkts   = 0;
static uint32_t Rx_Bad_Crc     = 0;
static uint32_t Rx_Short_Frames = 0;
static uint32_t Rx_Last_Seq    = 0u;

static uint32_t Rx_Vid_Total   = 0u;
static uint32_t Rx_Lost_Total  = 0u;
static uint32_t Rx_Seq_Max     = 0u;   /* highest seq the sender stamped     */
static uint32_t Rx_Bad_Total   = 0u;   /* CRC failures, cumulative           */

static uint32_t Rx_Stall_Cnt   = 0u;

#if (VID_RX_TRACE != 0u)
static uint32_t Rx_Trace_Cnt   = 0u;
#endif
static uint8_t  Rx_Have_Last   = 0u;
static uint32_t Rx_Last_Print_Tick = 0;
static uint32_t Ap_Ping_Tick   = 0u;
static uint32_t Ap_Chan_Tick   = 0u;
static uint32_t Ap_Env_Tick    = 0u;

static uint32_t Ap_Probe_Tick  = 0u;
static uint32_t Ap_Probe_Seq   = 0u;
static uint32_t Ap_Probe_Out   = 0u;

static uint8_t Ap_Bcast_Mac[6] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};

static uint8_t  Rx_First_Logged = 0u;

static uint8_t  Rx_Other_Logged = 0u;

static uint32_t Ap_Poll_Cnt = 0u;
static uint32_t Ap_Int_Cnt  = 0u;
static uint32_t Ap_Read_Cnt = 0u;
#endif

/**
 * @name    delay_ms
 * @brief   Millisecond delay driven by sys_now, does not block SysTick.
 * @param   ms: delay length in milliseconds.
 * @retval  None
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = sys_now();
    while ((sys_now() - start) < ms)
    {
    }
}

/**
 * @name    hgic_platform_raw_send
 * @brief   Platform hook that pushes a raw frame onto the SDIO link.
 * @param   data: pointer to the frame buffer.
 * @param   len: frame length in bytes.
 * @retval  Number of bytes written, or a negative value on error.
 */
int hgic_platform_raw_send(unsigned char *data, unsigned int len)
{
    return hgic_sdspi_write(0, data, len);
}

/**
 * @name    App_WaitFwInfo
 * @brief   Request the firmware version and poll the link until the module replies.
 * @param   None
 * @retval  None
 */
void App_WaitFwInfo(void)
{
    log_info("[INIT] get fwinfo...\r\n");
    hgic_raw_get_fwinfo();

    uint32_t fwinfo_start = sys_now();
    while ((sys_now() - fwinfo_start) < 3000u)
    {
        if (g_r900pnr_spi_rx_flag ||
            GPIO_ReadInputDataBit(R900PNR_SPI_INT_PORT, R900PNR_SPI_INT_PIN) == 0)
        {
            g_r900pnr_spi_rx_flag = 0;
            int read_len = hgic_sdspi_read(0, Spi_Rx_Buf, sizeof(Spi_Rx_Buf), 0);
            if (read_len > 0)
            {
                uint8_t *p_buf = Spi_Rx_Buf;
                uint32_t p_len = (uint32_t)read_len;
                hgic_raw_rx(&p_buf, &p_len);
            }
        }
    }
}

/**
 * @name    App_WiFiConfigure
 * @brief   Apply the SSID, key, band and channel settings for the compiled role.
 * @param   None
 * @retval  Byte count of the last command sent, or a negative value on failure.
 */
int App_WiFiConfigure(void)
{
    int ret = 0;

    hgic_raw_get_fwinfo();

#if (APP_WIFI_ROLE == APP_ROLE_AP)
    ret = hgic_raw_set_mode("ap");
    if (ret < 0)
    {
        delay_ms(500);
        ret = hgic_raw_set_mode("ap");
    }
    log_info("[CFG] set_mode(ap) = %d\r\n", ret);

    ret = hgic_raw_set_ssid(APP_WIFI_SSID);
    log_info("[CFG] set_ssid = %d\r\n", ret);

    ret = hgic_raw_set_key_mgmt("wpa2");
    log_info("[CFG] set_key_mgmt = %d\r\n", ret);

    ret = hgic_raw_set_wpa_psk(APP_WIFI_PSK);
    log_info("[CFG] set_wpa_psk = %d\r\n", ret);

#elif (APP_WIFI_ROLE == APP_ROLE_STA)
    ret = hgic_raw_set_mode("sta");
    if (ret < 0)
    {
        delay_ms(500);
        ret = hgic_raw_set_mode("sta");
    }
    log_info("[CFG] set_mode(sta) = %d\r\n", ret);

    ret = hgic_raw_set_ssid(APP_WIFI_SSID);
    log_info("[CFG] set_ssid = %d\r\n", ret);

    ret = hgic_raw_set_key_mgmt("wpa2");
    log_info("[CFG] set_key_mgmt = %d\r\n", ret);

    ret = hgic_raw_set_wpa_psk(APP_WIFI_PSK);
    log_info("[CFG] set_wpa_psk = %d\r\n", ret);
#endif

    /* Band: 908-924MHz, 8MHz bandwidth */
    uint16_t chan_list[] = APP_WIFI_CHAN_LIST;
    ret = hgic_raw_set_freq_range(chan_list[0], chan_list[2], APP_WIFI_BSS_BW);
    log_info("[CFG] set_freq_range = %d\r\n", ret);

    /* Bandwidth */
    ret = hgic_raw_set_bss_bw(APP_WIFI_BSS_BW);
    log_info("[CFG] set_bss_bw = %d\r\n", ret);

    /* Channel list */
    ret = hgic_raw_set_chan_list(chan_list, 3);
    log_info("[CFG] set_chan_list = %d\r\n", ret);

    /* Super power mode */
    ret = hgic_raw_set_supper_pwr(1);
    log_info("[CFG] set_supper_pwr = %d\r\n", ret);

    /* ACK timeout, for long-distance links */
    ret = hgic_raw_set_acktmo(10);
    log_info("[CFG] set_acktmo = %d\r\n", ret);

    /* Start the radio */
    ret = hgic_raw_open();
    log_info("[CFG] open = %d\r\n", ret);

    /* Persist the configuration to flash */
    ret = hgic_raw_save();
    log_info("[CFG] save = %d\r\n", ret);

#if (APP_WIFI_ROLE == APP_ROLE_STA)
    /* Kick off association */
    ret = hgic_raw_start_assoc();
    log_info("[CFG] start_assoc = %d\r\n", ret);
    Assoc_Start_Tick = sys_now();
#endif

    return ret;
}

#if (APP_WIFI_ROLE == APP_ROLE_AP)
/**
 * @name    App_RxIsVideo
 * @brief   Decide whether a data frame is one of our whole video frames.
 * @param   data: received frame.
 * @param   len: frame length in bytes.
 * @retval  1 when the frame is the right length and carries the magic, else 0.
 */
static uint8_t App_RxIsVideo(const uint8_t *data, uint32_t len)
{
    uint16_t magic;

    if (len != VID_FRAME_LEN)
    {
        return 0u;
    }

    magic = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    if (magic != VID_FRAME_MAGIC)
    {
        return 0u;
    }

    return 1u;
}

/**
 * @name    App_RxSeq
 * @brief   Read the sequence number out of a video frame header.
 * @param   data: received frame, at least VID_FRAME_HDR_LEN bytes.
 * @retval  The sequence number the sender put in the frame.
 */
static uint32_t App_RxSeq(const uint8_t *data)
{
    return (uint32_t)data[2]
         | ((uint32_t)data[3] << 8)
         | ((uint32_t)data[4] << 16)
         | ((uint32_t)data[5] << 24);
}

/**
 * @name    App_RxNoticeOther
 * @brief   Log the first few data frames that are not whole video frames.
 * @param   data: received frame.
 * @param   len: frame length in bytes.
 * @retval  None
 * @note    Reading the head past len stays inside the receive buffer, those
 *          bytes are stale, same as the trace below. The cookie comes from the
 *          frame header the vendor layer just parsed; it is the sender's own
 *          transmit counter, so it says whether these frames share a counter
 *          with ours.
 */
static void App_RxNoticeOther(const uint8_t *data, uint32_t len)
{
    if (Rx_Other_Logged >= 8u)
    {
        return;
    }
    Rx_Other_Logged++;

    log_info("[RX OTHER] len=%d cookie=%d head=%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
             (int)len, (int)hgic.rxcookie,
             data[0], data[1], data[2], data[3],
             data[4], data[5], data[6], data[7]);
}

/**
 * @name    App_RxCountLoss
 * @brief   Fold one received frame header into the loss statistics.
 * @param   data: received frame, at least VID_FRAME_HDR_LEN bytes.
 * @param   len: frame length in bytes.
 * @retval  None
 * @note    Frames that arrive out of order just widen the next gap, the
 *          counter cannot tell reordering from loss.
 */
static void App_RxCountLoss(const uint8_t *data, uint32_t len)
{
    uint16_t magic;
    uint32_t seq;

    if (len < VID_FRAME_HDR_LEN)
    {
        return;
    }

    magic = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    if (magic != VID_FRAME_MAGIC)
    {
        return;
    }

    seq = App_RxSeq(data);

    /* A jump of N means N-1 frames never showed up */
    if ((Rx_Have_Last != 0u) && (seq > (Rx_Last_Seq + 1u)))
    {
        Rx_Lost_Pkts  += (seq - Rx_Last_Seq - 1u);
        Rx_Lost_Total += (seq - Rx_Last_Seq - 1u);
    }

    if (seq > Rx_Seq_Max)
    {
        Rx_Seq_Max = seq;
    }

    Rx_Last_Seq  = seq;
    Rx_Have_Last = 1u;
}

#if (VID_RX_TRACE != 0u)
/**
 * @name    App_RxTraceFrame
 * @brief   Print the fields of one received frame, the first few only.
 * @param   data: received frame, at least VID_FRAME_HDR_LEN bytes.
 * @param   len: frame length in bytes.
 * @param   Crc_Field: CRC value the frame carries.
 * @retval  None
 * @note    The CRC is computed over a whole payload even when len says the
 *          frame is short, so the log shows whether the data is all there and
 *          only len is wrong, or the tail really is missing. Reading past len
 *          stays inside the receive buffer, those bytes are stale.
 */
static void App_RxTraceFrame(const uint8_t *data, uint32_t len,
                             uint32_t Crc_Field)
{
    uint32_t crc_calc;

    if (Rx_Trace_Cnt >= VID_RX_TRACE_FRAMES)
    {
        return;
    }
    Rx_Trace_Cnt++;

    crc_calc = App_Crc32(&data[VID_FRAME_HDR_LEN],
                                   VID_FRAME_PAYLOAD_LEN);

    log_info("[RX TRC] len=%d field=%08X calc=%08X tail=%02X%02X%02X%02X\r\n",
             (int)len, (unsigned int)Crc_Field, (unsigned int)crc_calc,
             data[VID_FRAME_LEN - 4u], data[VID_FRAME_LEN - 3u],
             data[VID_FRAME_LEN - 2u], data[VID_FRAME_LEN - 1u]);
}
#endif

/**
 * @name    App_RxCheckCrc
 * @brief   Verify the CRC32 a received frame carries over its payload.
 * @param   data: received frame, at least VID_FRAME_HDR_LEN bytes.
 * @param   len: frame length in bytes.
 * @retval  1 when the payload is intact, 0 when it must not be forwarded.
 * @note    A frame too short to hold a whole payload is counted apart from a
 *          real mismatch: nothing was checked, so calling it bad would blame
 *          the wire for a length problem. Either way the caller has to refuse
 *          the payload - a decoder handed bytes already known to be wrong
 *          turns one damaged frame into a stream it cannot tell from a good
 *          one, which is worse than the hole left by dropping it.
 */
static uint8_t App_RxCheckCrc(const uint8_t *data, uint32_t len)
{
    uint32_t magic;
    uint32_t crc_field;
    uint32_t crc_calc;
    uint8_t  pass = 1u;

    if (len < VID_FRAME_HDR_LEN)
    {
        return 0u;
    }

    magic = (uint32_t)data[0] | ((uint32_t)data[1] << 8);
    if (magic != VID_FRAME_MAGIC)
    {
        return 0u;
    }

    crc_field = (uint32_t)data[8]
              | ((uint32_t)data[9] << 8)
              | ((uint32_t)data[10] << 16)
              | ((uint32_t)data[11] << 24);

    if (len < VID_FRAME_LEN)
    {
        Rx_Short_Frames++;
        pass = 0u;
    }
    else
    {
        crc_calc = App_Crc32(&data[VID_FRAME_HDR_LEN],
                                       VID_FRAME_PAYLOAD_LEN);
        if (crc_calc != crc_field)
        {
            Rx_Bad_Crc++;
            Rx_Bad_Total++;
            pass = 0u;
        }
    }

#if (VID_RX_TRACE != 0u)
    App_RxTraceFrame(data, len, crc_field);
#endif

    return pass;
}

/**
 * @name    App_RingWrite
 * @brief   Append bytes to the receive ring, overwriting the oldest if full.
 * @param   pSrc: bytes to append.
 * @param   Len: number of bytes to append.
 * @retval  None
 * @note    Never blocks and never fails: a full ring loses its oldest bytes
 *          instead, which is what keeps a slow consumer from stalling the
 *          receive path. Every byte lost that way is counted in
 *          Rx_Ring_Dropped, which is a different fault from a frame lost over
 *          the air and has to be told apart from it.
 */
static void App_RingWrite(const uint8_t *pSrc, uint32_t Len)
{
    uint32_t free;
    uint32_t first;

    if (Len > VID_RX_RING_BYTES)
    {
        return;
    }

    free = VID_RX_RING_BYTES - Rx_Ring_Count;
    if (Len > free)
    {
        uint32_t shortage = Len - free;

        Rx_Ring_Rd = (Rx_Ring_Rd + shortage) % VID_RX_RING_BYTES;
        Rx_Ring_Count -= shortage;
        Rx_Ring_Dropped += shortage;
    }

    first = VID_RX_RING_BYTES - Rx_Ring_Wr;
    if (first > Len)
    {
        first = Len;
    }
    memcpy(&Rx_Ring[Rx_Ring_Wr], pSrc, first);
    memcpy(Rx_Ring, &pSrc[first], Len - first);

    Rx_Ring_Wr = (Rx_Ring_Wr + Len) % VID_RX_RING_BYTES;
    Rx_Ring_Count += Len;
    Rx_Ring_Written += Len;
}

/**
 * @name    App_VideoRxAvailable
 * @brief   How many bytes are readable right now.
 * @param   None
 * @retval  Byte count, always less than the buffer size.
 */
uint32_t App_VideoRxAvailable(void)
{
    return Rx_Ring_Count;
}

/**
 * @name    App_VideoRxRead
 * @brief   Take the oldest received bytes out of the buffer.
 * @param   pBuf: destination, must hold MaxLen bytes.
 * @param   MaxLen: most bytes to take this call.
 * @retval  Bytes actually taken, 0 when nothing is available.
 */
uint32_t App_VideoRxRead(uint8_t *pBuf, uint32_t MaxLen)
{
    uint32_t n;
    uint32_t first;

    if ((pBuf == NULL) || (MaxLen == 0u))
    {
        return 0u;
    }

    n = Rx_Ring_Count;
    if (n > MaxLen)
    {
        n = MaxLen;
    }
    if (n == 0u)
    {
        return 0u;
    }

    first = VID_RX_RING_BYTES - Rx_Ring_Rd;
    if (first > n)
    {
        first = n;
    }
    memcpy(pBuf, &Rx_Ring[Rx_Ring_Rd], first);
    memcpy(&pBuf[first], Rx_Ring, n - first);

    Rx_Ring_Rd = (Rx_Ring_Rd + n) % VID_RX_RING_BYTES;
    Rx_Ring_Count -= n;
    Rx_Ring_Read += n;

    return n;
}

/**
 * @name    App_VideoRxGetStats
 * @brief   Read the running counters.
 * @param   pStats: destination for the counters.
 * @retval  None
 */
void App_VideoRxGetStats(VideoRxStats_t *pStats)
{
    uint32_t lost;

    if (pStats == NULL)
    {
        return;
    }

    /* Frames the sender stamped but that never turned up: the highest seq
     * seen, plus one for the frame numbered zero, less the frames that did
     * arrive. Counting arrivals against the numbering is the reliable way -
     * summing the gaps between consecutive seq values counts reordering and
     * duplicate delivery as loss and overstates it by up to sixteen times. */
    lost = 0u;
    if ((Rx_Vid_Total != 0u) && (Rx_Seq_Max >= (Rx_Vid_Total - 1u)))
    {
        lost = (Rx_Seq_Max + 1u) - Rx_Vid_Total;
    }

    pStats->TotalBytes   = Rx_Ring_Written;
    pStats->ReadBytes    = Rx_Ring_Read;
    pStats->DroppedBytes = Rx_Ring_Dropped;
    pStats->LostFrames   = lost;
    pStats->BadCrcFrames = Rx_Bad_Total;
}
#endif

/**
 * @name    App_RxDataHandler
 * @brief   Accumulate received frames and log the throughput once per second.
 * @param   data: pointer to the received payload.
 * @param   len: payload length in bytes.
 * @retval  None
 */
void App_RxDataHandler(uint8_t *data, uint32_t len)
{
#if (APP_WIFI_ROLE == APP_ROLE_AP)
    const uint8_t *vid_data;
    uint32_t       vid_len;

    Rx_Total_Pkts++;

    if (len <= sizeof(struct hgic_eth_hdr))
    {
        Rx_Other_Pkts++;
        return;
    }
    vid_data = &data[sizeof(struct hgic_eth_hdr)];
    vid_len  = len - (uint32_t)sizeof(struct hgic_eth_hdr);

    if (App_RxIsVideo(vid_data, vid_len) == 0u)
    {
        Rx_Other_Pkts++;
        App_RxNoticeOther(data, len);
        return;
    }

    Rx_Vid_Pkts++;
    Rx_Vid_Total++;
    Rx_Vid_Bytes += VID_FRAME_LEN;

    if (Rx_First_Logged == 0u)
    {
        Rx_First_Logged = 1u;
        log_info("[RX] first video frame seq=%d len=%d\r\n",
                 (int)App_RxSeq(vid_data), (int)vid_len);
    }

    App_RxCountLoss(vid_data, vid_len);
    if (App_RxCheckCrc(vid_data, vid_len) != 0u)
    {
        App_RingWrite(&vid_data[VID_FRAME_HDR_LEN], VID_FRAME_PAYLOAD_LEN);
    }
#endif
}

#if (APP_WIFI_ROLE == APP_ROLE_AP)
/**
 * @name    App_ApAirProbeTick
 * @brief   Broadcast one small probe frame at APP_AP_PROBE_MS.
 * @param   None
 * @retval  None
 * @note    The probe exists because every counter on this board is fed by
 *          frames arriving from the sender, so a stall here is equally
 *          consistent with the sender having gone quiet and with this end
 *          having gone deaf. A frame this board originates breaks the
 *          symmetry: the sender sees it or it does not, and that is a reading
 *          neither board can produce on its own.
 */
static void App_ApAirProbeTick(void)
{
    uint8_t  frame[PRB_FRAME_LEN];
    uint32_t now = sys_now();

    if ((now - Ap_Probe_Tick) < APP_AP_PROBE_MS)
    {
        return;
    }
    Ap_Probe_Tick = now;

    memset(frame, 0, sizeof(frame));
    frame[0] = (uint8_t)(PRB_FRAME_MAGIC & 0xFFu);
    frame[1] = (uint8_t)((PRB_FRAME_MAGIC >> 8) & 0xFFu);
    frame[2] = (uint8_t)(Ap_Probe_Seq);
    frame[3] = (uint8_t)(Ap_Probe_Seq >> 8);
    frame[4] = (uint8_t)(Ap_Probe_Seq >> 16);
    frame[5] = (uint8_t)(Ap_Probe_Seq >> 24);

    if (hgic_raw_send(Ap_Bcast_Mac, frame, PRB_FRAME_LEN) > 0)
    {
        Ap_Probe_Seq++;
        Ap_Probe_Out++;
    }
}

/**
 * @name    App_RxStatsTick
 * @brief   Log the receive statistics once a second, whether or not anything arrived.
 * @param   None
 * @retval  None
 * @note    This runs on a timer from the poll loop, not from the frame handler.
 *          Printing only when a frame arrives makes a receiver that stopped
 *          being serviced produce no output at all, which reads exactly like a
 *          receiver with nothing to report. poll/int/rd name the stage that
 *          stopped: poll is the loop turning, int is the module asking for
 *          attention, rd is the read returning bytes. pkts counts every data
 *          frame, vid only the ones that are ours, oth the rest. rate covers
 *          the video bytes alone.
 */
static void App_RxStatsTick(void)
{
    uint32_t now;
    uint32_t win;
    uint32_t rate;

    Ap_Poll_Cnt++;

    now = sys_now();
    if ((now - Rx_Last_Print_Tick) < 1000u)
    {
        return;
    }

    win  = now - Rx_Last_Print_Tick;
    rate = (Rx_Vid_Bytes * 8u) / win;

    if ((now - Ap_Ping_Tick) >= APP_AP_PING_MS)
    {
        Ap_Ping_Tick = now;
        hgic_raw_get_sta_list();
    }

    if ((now - Ap_Chan_Tick) >= APP_AP_CHAN_PROBE_MS)
    {
        Ap_Chan_Tick = now;
        hgic_raw_get_center_freq();
    }

    if ((now - Ap_Env_Tick) >= APP_RF_ENV_PROBE_MS)
    {
        Ap_Env_Tick = now;
        hgic_raw_get_disassoc_reason();
    }

    if ((Rx_Vid_Total > 0u) && (Rx_Vid_Pkts == 0u))
    {
        Rx_Stall_Cnt++;
    }

    log_info("[RX STAT] pkts=%d vid=%d oth=%d lost=%d bad=%d short=%d"
             " rate=%d Kbps seq=%d win=%d ms poll=%d int=%d rd=%d"
             " vidT=%d lostT=%d stall=%d aprb=%d"
             " rtot=%d rread=%d rdrop=%d ravail=%d\r\n",
             Rx_Total_Pkts, Rx_Vid_Pkts, Rx_Other_Pkts, Rx_Lost_Pkts,
             Rx_Bad_Crc, Rx_Short_Frames, rate, (int)Rx_Last_Seq, win,
             Ap_Poll_Cnt, Ap_Int_Cnt, Ap_Read_Cnt,
             Rx_Vid_Total, Rx_Lost_Total, Rx_Stall_Cnt, Ap_Probe_Out,
             Rx_Ring_Written, Rx_Ring_Read, Rx_Ring_Dropped, Rx_Ring_Count);

    Rx_Total_Pkts      = 0;
    Rx_Vid_Pkts        = 0;
    Rx_Other_Pkts      = 0;
    Rx_Vid_Bytes       = 0;
    Rx_Lost_Pkts       = 0;
    Rx_Bad_Crc         = 0;
    Rx_Short_Frames    = 0;
    Ap_Poll_Cnt        = 0;
    Ap_Int_Cnt         = 0;
    Ap_Read_Cnt        = 0;
    Ap_Probe_Out       = 0;
    Rx_Last_Print_Tick = now;

}
#endif

/**
 * @name    App_EventHandler
 * @brief   Track the association state reported by the module.
 * @param   event_id: event identifier from the raw layer.
 * @param   value: event payload.
 * @retval  None
 */
void App_EventHandler(uint8_t event_id, int16_t value)
{
#if (APP_WIFI_ROLE == APP_ROLE_STA)
    if (event_id == HGIC_EVENT_CONECTED)
    {
        Wifi_Connected = 1;
        log_info("[EVENT] WiFi connected!\r\n");
    }
    else if (event_id == HGIC_EVENT_DISCONECTED)
    {
        Wifi_Connected = 0;
        log_info("[EVENT] WiFi disconnected!\r\n");
    }
#endif
}

#if (APP_WIFI_ROLE == APP_ROLE_STA)
/**
 * @name    App_TxIsProbe
 * @brief   Say whether the probe magic sits at a given offset in a frame.
 * @param   data: received frame.
 * @param   len: frame length in bytes.
 * @param   off: offset to look at.
 * @retval  1 when the magic is there, else 0.
 */
static uint8_t App_TxIsProbe(const uint8_t *data, uint32_t len, uint32_t off)
{
    uint16_t magic;

    if (len < (off + 2u))
    {
        return 0u;
    }

    magic = (uint16_t)((uint16_t)data[off] | ((uint16_t)data[off + 1u] << 8));

    return (magic == PRB_FRAME_MAGIC) ? 1u : 0u;
}

/**
 * @name    App_TxAirCount
 * @brief   Classify one data frame that arrived over the air.
 * @param   data: received frame.
 * @param   len: frame length in bytes.
 * @retval  None
 * @note    This end hosts no receiver of its own, so every data frame that
 *          reaches here crossed the air in the AP -> STA direction whatever
 *          else is true of it. Counting them is the half of the direction
 *          probe that can only be measured here.
 */
static void App_TxAirCount(const uint8_t *data, uint32_t len)
{
    Tx_Air_Cnt++;

    if (App_TxIsProbe(data, len, sizeof(struct hgic_eth_hdr)) == 0u)
    {
        return;
    }

    Tx_Air_Probe++;

    if (Peer_Mac_Valid == 0u)
    {
        Peer_Mac_Valid = 1u;
        memcpy(Peer_Mac, &data[6], 6u);
        log_info("[TX] peer mac "MACSTR"\r\n", MAC2STR(Peer_Mac));
    }
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
 * @param   None
 * @retval  None
 * @note    A token bucket holds the rate at VID_TARGET_KBPS. Sending flat out
 *          would overrun the measured link and lose frames.
 */
void App_VideoStreamSend(void)
{
    uint32_t now;
    uint32_t elapsed;

    if (!Tx_Started)
    {
        if (Wifi_Connected)
        {
            log_info("[TX] WiFi connected\r\n");
        }
        else if ((sys_now() - Assoc_Start_Tick) < APP_ASSOC_TIMEOUT_MS)
        {
            return;
        }
        else
        {
            log_info("[TX] assoc timeout, start sending anyway\r\n");

            hgic_raw_get_disassoc_reason();
        }

        Tx_Started = 1;

        if (App_VideoStreamOpen() == VIDEO_OK)
        {
            log_info("[TX] streaming the card payload\r\n");
        }
        else
        {
            log_info("[TX] no usable payload on the card, not sending\r\n");
        }
    }

    if (Vid_Ready == 0u)
    {
        return;
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
 * @name    App_TxStatsTick
 * @brief   Log the send statistics once a second and ask the module for its state.
 * @param   None
 * @retval  None
 * @note    This runs on a timer from the poll loop, not from the send path, so
 *          it keeps reporting while the stream is stalled and before it starts.
 *          The rate is measured over a real window, not assumed to be a second.
 *
 *          conn comes from the vendor layer's own association state, which is
 *          the only view of it we have: hgic_raw_rx drops the result of
 *          hgic_raw_rx_event and falls through to return IGNORE, so the EVENT
 *          branch below never fires and App_EventHandler never runs. That does
 *          not make the event path blind - hgic_raw_rx_event itself still runs
 *          and prints unconditionally, so a connect or disconnect would show up
 *          as the vendor's own "STA:xx connected" line whatever we do. No such
 *          line has ever appeared on either board, so the module has raised no
 *          association event while we were listening, and the polled value is
 *          the one to trust. tbr is the module's own reported TX bitrate, and
 *          rx counts the SDIO reads that came back with bytes, so rx falling to
 *          zero says the module has stopped answering at all.
 */
static void App_TxStatsTick(void)
{
    uint32_t now;
    uint32_t win;
    uint32_t rate;

    now = sys_now();
    if ((now - Tx_Last_Print_Tick) < 1000u)
    {
        return;
    }

    win  = now - Tx_Last_Print_Tick;
    rate = (Tx_Total_Bytes * 8u) / win;

    hgic_raw_get_connect_state();

    hgic_raw_get_temperature();

    log_info("[TX STAT] pkts=%d bytes=%d rate=%d Kbps fail=%d win=%d ms"
             " seq=%d conn=%d rssi=%d tbr=%d rx=%d air=%d prb=%d\r\n",
             Tx_Total_Pkts, Tx_Total_Bytes, rate, Tx_Fail_Cnt, win,
             (int)Vid_Seq,
             (int)hgic.status.conn_state, (int)hgic.status.rssi,
             (int)hgic.status.tx_bitrate, Tx_Rx_Cnt,
             Tx_Air_Cnt, Tx_Air_Probe);

    Tx_Total_Bytes     = 0;
    Tx_Total_Pkts      = 0;
    Tx_Fail_Cnt        = 0;
    Tx_Rx_Cnt          = 0;
    Tx_Air_Cnt         = 0;
    Tx_Air_Probe       = 0;
    Tx_Last_Print_Tick = now;
}
#endif

/**
 * @name    App_R900PnrLinkCheck
 * @brief   Probe the SDIO link periodically and re-initialize it if it went down.
 * @param   None
 * @retval  None
 */
void App_R900PnrLinkCheck(void)
{
    static uint32_t Link_Check_Tick = 0U;

    if ((sys_now() - Link_Check_Tick) >= APP_ALIVE_PERIOD_MS)
    {
        Link_Check_Tick = sys_now();

        /* Module falls back to SD mode on a comms fault */
        if (hgic_sdspi_detect_alive(0) < 0)
        {
            log_error("[R900PNR] SDIO link lost, re-init\r\n");
            hgic_sdspi_init(0);
        }
    }
}

/**
 * @name    App_R900PnrPoll
 * @brief   Service one radio frame per call; runs as the main-loop background task.
 * @param   None
 * @retval  None
 */
void App_R900PnrPoll(void)
{
    /* 1. Link health check */
    App_R900PnrLinkCheck();

    /* 2. Receive one frame when the module signals or INT is held low */
    if (g_r900pnr_spi_rx_flag ||
        GPIO_ReadInputDataBit(R900PNR_SPI_INT_PORT, R900PNR_SPI_INT_PIN) == 0)
    {
#if (APP_WIFI_ROLE == APP_ROLE_AP)
        Ap_Int_Cnt++;
#endif
        g_r900pnr_spi_rx_flag = 0;
        int read_len = hgic_sdspi_read(0, Spi_Rx_Buf, sizeof(Spi_Rx_Buf), 0);
        if (read_len > 0)
        {
#if (APP_WIFI_ROLE == APP_ROLE_STA)
            Tx_Rx_Cnt++;
#endif
            uint8_t *p_buf = Spi_Rx_Buf;
            uint32_t p_len = (uint32_t)read_len;
            HGIC_RAW_RX_TYPE rx_type = hgic_raw_rx(&p_buf, &p_len);
#if (APP_WIFI_ROLE == APP_ROLE_AP)
            Ap_Read_Cnt++;
#endif

            if (rx_type == HGIC_RAW_RX_TYPE_DATA && p_len > 0)
            {
#if (APP_WIFI_ROLE == APP_ROLE_AP)
                App_RxDataHandler(p_buf, p_len);
#else
                App_TxAirCount(p_buf, p_len);
#endif
            }
            else if (rx_type == HGIC_RAW_RX_TYPE_EVENT)
            {
                /* Event format: event_id(1B) + value(2B) */
                if (p_len >= 1)
                {
                    uint8_t event_id = p_buf[0];
                    int16_t value = (p_len >= 3) ? (int16_t)(p_buf[1] | (p_buf[2] << 8)) : 0;
                    App_EventHandler(event_id, value);
                }
            }
        }
    }

    /* 3. Send path, STA only */
#if (APP_WIFI_ROLE == APP_ROLE_STA)
    App_VideoStreamSend();
#endif

#if (APP_WIFI_ROLE == APP_ROLE_AP)
    App_ApAirProbeTick();
    App_RxStatsTick();
#else
    App_TxStatsTick();
#endif
}

/**
 * @name    r900pnr_wifi_module_init
 * @brief   Initialize the SPI interface and the SDIO link to the R900PNR module.
 * @param   None
 * @retval  R900PNR_OK on success, or a negative r900pnr_error_t value on failure.
 */
int r900pnr_wifi_module_init(void)
{
    int ret = 0;

    /* 1. SPI interface */
    if (R900PNR_SPI_Init() != 0)
		{
        log_error("[R900PNR] SPI init failed");
        return R900PNR_ERR_SPI_INIT;
    }
    log_info("[R900PNR] SPI init success");

    /* 2. SDIO interface, up to SDIO_INIT_MAX_RETRY_COUNT attempts */
    ret = hgic_sdspi_init(0);
    for (int attempt = 1; (ret != 0) && (attempt < SDIO_INIT_MAX_RETRY_COUNT); attempt++)
		{
			log_info("[R900PNR] SDIO init failed (attempt %d/%d), module not ready, retry in %d ms",
							 attempt, SDIO_INIT_MAX_RETRY_COUNT, SDIO_INIT_RETRY_DELAY_MS);
			delay_ms(SDIO_INIT_RETRY_DELAY_MS);
			ret = hgic_sdspi_init(0);
    }

    if (ret != 0)
		{
			log_info("[R900PNR] SDIO init failed after %d attempts", SDIO_INIT_MAX_RETRY_COUNT);
			return R900PNR_ERR_SDIO_INIT;
    }
    log_info("[R900PNR] SDIO init success");

    /* 3. Wait for the bus to settle */
    delay_ms(SDIO_STABLE_DELAY_MS);
    log_info("[R900PNR] SDIO bus stable after %d ms", SDIO_STABLE_DELAY_MS);

    log_info("[R900PNR] Configure Wi-Fi...");

    return R900PNR_OK;
}
