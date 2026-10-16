#include "App_OTA_Handle.h"
#include <stdint.h>
#include "stdio.h"
#include "misc.h"
#include "Mcal_Uart.h"
volatile uint8_t ota_request_flag = 0;

/* How long to let the reboot wait for the message above it to leave the chip.
 * The wait is a poll loop, so this is in iterations rather than milliseconds:
 * generously sized, because it costs nothing to wait longer than needed and the
 * loop exits the moment the transmitter is idle. It exists at all because the
 * console is now driven by DMA, so printf() returns long before the bytes are
 * on the wire -- without this the reboot would cut the message off and the
 * operator would never learn whether the command had been seen. */
#define OTA_TX_DRAIN_SPIN        (2000000u)

/**
 * @name    ota_cmd_feed
 * @brief   Feed one received character to the "UPDATE" command matcher.
 * @param   ch: the character received on the console UART.
 * @retval  1 when the full command followed by CR or LF has been matched, else 0.
 */
uint8_t ota_cmd_feed(uint8_t ch)
{
    static const char cmd[] = OTA_CMD_STRING;
    static uint8_t match_idx = 0;

    if (match_idx >= (sizeof(cmd) - 1u))
    {
        if ((ch == '\r') || (ch == '\n'))
        {
            match_idx = 0;
            return 1;
        }
        match_idx = (ch == (uint8_t)cmd[0]) ? 1u : 0u;
        return 0;
    }

    if (ch == (uint8_t)cmd[match_idx])
    {
        match_idx++;
    }
    else
    {
        match_idx = (ch == (uint8_t)cmd[0]) ? 1u : 0u;
    }
    return 0;
}

/**
 * @name    ota_request_reset
 * @brief   Publish the OTA flag to the agreed SRAM address and reboot.
 * @param   None
 * @retval  None, does not return.
 * @note    The wait before the reset is the one place in this firmware that is
 *          allowed to block on the transmitter, and it is allowed because there
 *          is no next iteration to be late for. Everywhere else the console is
 *          fire-and-forget; here the message is the last thing the board will
 *          ever say, and a message that dies with the reset is worse than no
 *          message, because the absence of it looks like the command was never
 *          received. The wait is bounded, so a wedged transmitter still reboots.
 */
void ota_request_reset(void)
{
    printf("\r\n OTA update request received, reboot to bootloader...\r\n");

    (void)Mcal_Uart_TxWaitDone(OTA_TX_DRAIN_SPIN);      /* 等发完再复位 */

    *(volatile uint32_t *)OTA_FLAG_ADDR = OTA_FLAG_MAGIC;
    SCB_CleanDCache();  
    __set_FAULTMASK(1);
    __NVIC_SystemReset();
}


/**
 * @name    OTA_UpdateCheck_Handle
 * @brief   Reboot into the bootloader when an OTA request is pending.
 * @param   None
 * @retval  None
 */
void OTA_UpdateCheck_Handle(void)
{
	if (ota_request_flag) { ota_request_reset(); }
}
