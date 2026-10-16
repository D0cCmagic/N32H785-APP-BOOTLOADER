/**
 * @file    App_R900PNR_Handle.c
 * @brief   Radio service: polls the module, keeps its counters, configures it.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.0.0
 *
 * @note    Split out of a file that had grown to carry this, the video send
 *          path and the video receive path at once. What is left is the link:
 *          the poll, the link health check, the Wi-Fi configuration, the
 *          counters the statistics line prints, and the module bring-up.
 * @note    The video send path is called from here once per poll, and is told
 *          only what it needs about the link -- whether the module has
 *          associated and how long ago association was asked for. Everything
 *          else about the send path stays in its own module.
 * @note    This build is the sender. The receive half of the old dual-role code
 *          went with the AP build.
 */

#include "App_R900PNR_Handle.h"
#include "App_VideoTx_Handle.h"
#include <string.h>

/* Module-private statics */
static uint8_t Spi_Rx_Buf[2048u];

static uint8_t  Wifi_Connected = 0;
static uint32_t Assoc_Start_Tick = 0;
static uint32_t Tx_Rx_Cnt      = 0;
static uint32_t Tx_Last_Print_Tick = 0;
static uint32_t Tx_Air_Cnt     = 0u;
static uint32_t Tx_Air_Probe   = 0u;

static uint8_t  Peer_Mac[6]    = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
static uint8_t  Peer_Mac_Valid = 0u;

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
 * @brief   Apply the SSID, key, band and channel settings, then start the link.
 * @param   None
 * @retval  Byte count of the last command sent, or a negative value on failure.
 */
int App_WiFiConfigure(void)
{
    int ret = 0;

    hgic_raw_get_fwinfo();

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

    /* Band: 908-924MHz, 4MHz bandwidth */
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

#if (VID_RF_TX_MCS != VID_RF_UNCHANGED)
    ret = hgic_raw_set_tx_mcs((char)VID_RF_TX_MCS);
    log_info("[RF] tx_mcs=%d ret=%d\r\n", (int)VID_RF_TX_MCS, ret);
#endif

#if (VID_RF_MAX_TXCNT != VID_RF_UNCHANGED)
    ret = hgic_raw_set_max_txcnt((char)VID_RF_MAX_TXCNT);
    log_info("[RF] max_txcnt=%d ret=%d\r\n", (int)VID_RF_MAX_TXCNT, ret);
#endif

#if (VID_RF_TX_MCS != VID_RF_UNCHANGED) || (VID_RF_MAX_TXCNT != VID_RF_UNCHANGED)
    /* A candidate must not reach the module's flash until it has proven itself */
    ret = hgic_raw_set_auto_save(0);
    log_info("[RF] auto_save off = %d\r\n", ret);
#endif

    /* Start the radio */
    ret = hgic_raw_open();
    log_info("[CFG] open = %d\r\n", ret);

#if ((VID_RF_TX_MCS != VID_RF_UNCHANGED) || (VID_RF_MAX_TXCNT != VID_RF_UNCHANGED))
    log_info("[CFG] radio override in use, not saving to flash\r\n");
#else
    /* Persist the configuration to flash */
    ret = hgic_raw_save();
    log_info("[CFG] save = %d\r\n", ret);
#endif

    /* Kick off association */
    ret = hgic_raw_start_assoc();
    log_info("[CFG] start_assoc = %d\r\n", ret);
    Assoc_Start_Tick = sys_now();

    return ret;
}

/**
 * @name    App_EventHandler
 * @brief   Track the association state reported by the module.
 * @param   event_id: event identifier from the raw layer.
 * @param   value: event payload.
 * @retval  None
 * @note    Reached only if the vendor layer ever returns HGIC_RAW_RX_TYPE_EVENT;
 *          see the note on App_TxStatsTick for why it does not.
 */
void App_EventHandler(uint8_t event_id, int16_t value)
{
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
}

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
 * @note    The peer address learned here is handed to the send path, which is
 *          the only other module that needs it.
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
        App_VideoTx_SetPeer(Peer_Mac);
        log_info("[TX] peer mac "MACSTR"\r\n", MAC2STR(Peer_Mac));
    }
}

/**
 * @name    App_TxStatsTick
 * @brief   Log the send statistics every APP_STATS_PERIOD_MS and ask the module for its state.
 * @param   None
 * @retval  None
 * @note    This runs on a timer from the poll loop, not from the send path, so
 *          it keeps reporting while the stream is stalled and before it starts.
 *          The rate is measured over a real window, not assumed to be a second.
 *
 *          The temperature poll was dropped: hgic.status.temperature is read
 *          nowhere else in this application, and an overheat reaches us anyway
 *          as the driver's own unsolicited TEMPERATURE OVERTOP line.
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
 *
 *          The frame counters belong to the send path and arrive through its
 *          getter; reading them closes the window they cover.
 */
static void App_TxStatsTick(void)
{
    uint32_t now;
    uint32_t win;
    uint32_t rate;
    App_VideoTxStats_t vtx;

    now = sys_now();
    if ((now - Tx_Last_Print_Tick) < APP_STATS_PERIOD_MS)
    {
        return;
    }

    win  = now - Tx_Last_Print_Tick;
    App_VideoTxGetStats(&vtx);
    rate = (vtx.Bytes * 8u) / win;

    log_info("[TX STAT] pkts=%d bytes=%d rate=%d Kbps fail=%d win=%d ms"
             " seq=%d conn=%d rssi=%d tbr=%d rx=%d air=%d prb=%d\r\n",
             (int)vtx.Pkts, (int)vtx.Bytes, rate, (int)vtx.Fail, win,
             (int)vtx.Seq,
             (int)hgic.status.conn_state, (int)hgic.status.rssi,
             (int)hgic.status.tx_bitrate, Tx_Rx_Cnt,
             Tx_Air_Cnt, Tx_Air_Probe);

    Tx_Rx_Cnt          = 0;
    Tx_Air_Cnt         = 0;
    Tx_Air_Probe       = 0;
    Tx_Last_Print_Tick = now;
}

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
        g_r900pnr_spi_rx_flag = 0;
        int read_len = hgic_sdspi_read(0, Spi_Rx_Buf, sizeof(Spi_Rx_Buf), 0);
        if (read_len > 0)
        {
            Tx_Rx_Cnt++;
            uint8_t *p_buf = Spi_Rx_Buf;
            uint32_t p_len = (uint32_t)read_len;
            HGIC_RAW_RX_TYPE rx_type = hgic_raw_rx(&p_buf, &p_len);

            if (rx_type == HGIC_RAW_RX_TYPE_DATA && p_len > 0)
            {
                App_TxAirCount(p_buf, p_len);
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

    /* 3. Send path: told whether the link is up, and how long it has been since
     *    association was asked for, which is all it needs to decide to start. */
    App_VideoStreamSend(Wifi_Connected, sys_now() - Assoc_Start_Tick);

    /* 4. Statistics */
    App_TxStatsTick();
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
