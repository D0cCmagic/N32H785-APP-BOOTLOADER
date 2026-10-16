/**
 * @file    App_SdCard_Handle.h
 * @brief   SD card driver in SPI mode: bring-up and raw block read.
 * @author  D0cC
 * @date    2026-09-11
 * @version V1.1.0
 *
 * @note    Application layer. This is the SD protocol: command framing,
 *          the bring-up state machine, CSD decoding and the block read
 *          sequence. Every byte on the wire goes through the Mcal_SdSpi_*
 *          primitives underneath.
 *          There is no file system here on purpose: the video payload is read
 *          as raw 512 byte blocks straight off the card.
 *          Only read is implemented, the AP side never writes.
 *          SPI mode has no RCA and selects the card with chip select, so the
 *          SD bus mode commands CMD2, CMD3 and CMD7 are never issued here.
 */

#ifndef APP_SD_CARD_HANDLE_H
#define APP_SD_CARD_HANDLE_H

#include <stdint.h>
#include "Mcal_SdSpi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define Sd_GetTickMs()  SdSpi_GetTickMs()

/* One SD block, the only transfer unit this driver works in */
#define SD_BLOCK_SIZE                   (512u)

/**
 * @brief  Addressing scheme the card reported during bring-up.
 */
typedef enum
{
    SD_CARD_TYPE_UNKNOWN = 0,
    SD_CARD_TYPE_SD1,       /* SD v1.x, byte addressed      */
    SD_CARD_TYPE_SD2,       /* SD v2.x, byte addressed      */
    SD_CARD_TYPE_SDHC       /* SDHC / SDXC, block addressed */
} SdCard_Type_t;

/**
 * @brief  Driver return codes.
 */
typedef enum
{
    SD_CARD_OK = 0,                 /* Success                                   */
    SD_CARD_ERR_PARAM = -1,         /* Bad argument from the caller              */
    SD_CARD_ERR_NO_CARD = -2,       /* No card answered CMD0                     */
    SD_CARD_ERR_VOLTAGE = -3,       /* Card rejects the 2.7..3.6 V range         */
    SD_CARD_ERR_TIMEOUT = -4,       /* Card stayed busy past the timeout         */
    SD_CARD_ERR_RESPONSE = -5,      /* Unexpected or error response byte         */
    SD_CARD_ERR_TOKEN = -6,         /* Data token never arrived                  */
    SD_CARD_ERR_UNSUPPORTED = -7    /* Card type or CSD not handled              */
} SdCard_Status_t;

/**
 * @brief  Everything the application needs to know about the inserted card.
 */
typedef struct
{
    SdCard_Type_t Type;             /* Addressing scheme in use                  */
    uint32_t      BlockCount;       /* Total 512 byte blocks on the card         */
    uint32_t      CapacityKb;       /* Total capacity in kilobytes               */
    uint8_t       Csd[16];          /* Raw CSD, kept for the caller to inspect   */
} SdCard_Info_t;

/**
 * @name    App_SdCard_Init
 * @brief   Bring up SPI2 and run the full SD SPI bring-up sequence.
 * @param   None
 * @retval  SD_CARD_OK on success, a negative SdCard_Status_t otherwise.
 */
int App_SdCard_Init(void);

/**
 * @name    App_SdCard_GetInfo
 * @brief   Copy out the card description captured during bring-up.
 * @param   pInfo: destination, left untouched when it is NULL.
 * @retval  SD_CARD_OK on success, SD_CARD_ERR_PARAM when pInfo is NULL.
 */
int App_SdCard_GetInfo(SdCard_Info_t *pInfo);

/**
 * @name    App_SdCard_ReadBlocks
 * @brief   Read consecutive 512 byte blocks starting at the given block address.
 * @param   BlockAddr: first block, always a block number not a byte offset.
 * @param   pBuf: destination buffer, must hold BlockCount * 512 bytes.
 * @param   BlockCount: number of blocks to read, must not be zero.
 * @retval  SD_CARD_OK on success, a negative SdCard_Status_t otherwise.
 */
int App_SdCard_ReadBlocks(uint32_t BlockAddr, uint8_t *pBuf, uint32_t BlockCount);

#ifdef __cplusplus
}
#endif

#endif /* APP_SD_CARD_HANDLE_H */
