#ifndef __APP_R900PNR_HANDLE_H__
#define __APP_R900PNR_HANDLE_H__

/**
 * @file    App_R900PNR_Handle.h
 * @brief   Radio service: the R900PNR link, its configuration and its counters.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.0.0
 *
 * @note    Split out of a single module that also carried the video send path
 *          and the video receive path. The send path is now
 *          App_VideoTx_Handle.c, and this header no longer carries the frame
 *          format or the send pacing.
 * @note    This build is the sender. The receive half of the old dual-role code
 *          is gone with the AP build, which is not this project's any more.
 */

#include "stdint.h"
#include "n32h7xx.h"
#include "n32h7xx_gpio.h"
#include "r900pnr_spi_cfg.h"
#include "r900pnr_spi_api.h"
#include "hgic_raw.h"
#include "hgic_sdspi.h"
#include "log.h"

/* System tick in milliseconds, defined in main.c and incremented by SysTick */
uint32_t sys_now(void);
#define get_tick_ms()  sys_now()

/* Millisecond delay based on sys_now, does not block SysTick */
void delay_ms(uint32_t ms);

/* SPI receive flag, defined in r900pnr_spi_api.c */
extern volatile uint8_t g_r900pnr_spi_rx_flag;

/* R900PNR application settings */
#define APP_WIFI_SSID                   "hgic_ah_test"
#define APP_WIFI_PSK                    "12345678"
#define APP_WIFI_BSS_BW                 (4u)
#define APP_WIFI_CHAN_LIST              {9080u, 9160u, 9240u}

#define APP_ALIVE_PERIOD_MS             (100u)
#define APP_TX_BUF_SIZE                 (1500u)
#define APP_LINK_PING_MS                (1000u)

/* The probe frame the sender watches for, so that the far side's own traffic can
 * be told apart from everything else that arrives. */
#define PRB_FRAME_MAGIC                 (0x5250u)

/* Cadence for the periodic status line. Long enough to stay readable on a
 * console shared with the vendor driver own output, short enough that a link
 * that drops still shows up promptly.
 */
#define APP_STATS_PERIOD_MS             (5000u)

/*
 * Radio parameters applied before the link comes up.
 * VID_RF_UNCHANGED leaves a parameter at the module's own default.
 *
 * Neither setter has ever been called by this project and neither range is
 * documented, so change one macro at a time and reflash. A value the module
 * refuses shows up as the vendor's own "CMD:<id>, RESP:Fail[-524]" line, not
 * as the return code of the call. A value it accepts but that breaks the link
 * is cleared by pulling power, because the configuration is not written to the
 * module's flash while an override is in use: do not call hgic_raw_save()
 * until one setting has proven itself.
 */
#define VID_RF_UNCHANGED                (0xFFu)
#define VID_RF_TX_MCS                   (3u)
#define VID_RF_MAX_TXCNT                (8u)


#define R900PNR_SPI_INIT_RETRY_COUNT       (0)     /* SPI init is not retried, returns error */
#define SDIO_INIT_MAX_RETRY_COUNT          (5)     /* Max SDIO init attempts */
#define SDIO_INIT_RETRY_DELAY_MS           (1000)  /* Delay between SDIO init attempts, ms */
#define SDIO_STABLE_DELAY_MS               (200)   /* Bus settle delay after SDIO init, ms */


/* Return codes */
typedef enum {
    R900PNR_OK = 0,                 /**< Success */
    R900PNR_ERR_SPI_INIT = -1,      /**< SPI init failed */
    R900PNR_ERR_SDIO_INIT = -2,     /**< SDIO init failed */
} r900pnr_error_t;

/* Public API */
void App_EventHandler(uint8_t event_id, int16_t value);
int  App_WiFiConfigure(void);
void App_R900PnrPoll(void);
int  hgic_platform_raw_send(unsigned char *data, unsigned int len);
void App_WaitFwInfo(void);
void App_R900PnrLinkCheck(void);
int r900pnr_wifi_module_init(void);
#endif
