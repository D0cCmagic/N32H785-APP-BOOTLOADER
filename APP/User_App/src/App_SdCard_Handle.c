/**
 * @file    App_SdCard_Handle.c
 * @brief   SD card driver in SPI mode: bring-up and raw block read.
 * @author  D0cC
 * @date    2026-09-11
 * @version V1.1.0
 *
 * @note    Targets the N32H785 dev board, M7 core, standard peripheral library.
 *          The card is driven on SPI2, SPI1 belongs to the R900PNR module.
 *          Everything below the byte level lives in Mcal_SdSpi.c, this file
 *          only speaks the SD command protocol on top of it.
 *          Bring-up follows the SPI mode sequence of the SD physical layer
 *          spec: CMD0, CMD8, ACMD41, CMD58, CMD9, CMD16.
 *          CMD2, CMD3 and CMD7 belong to the SD bus mode and are illegal here.
 */

#include "App_SdCard_Handle.h"
#include "log.h"

/* Command indices used during bring-up and read-out
 * CMD2 / CMD3 / CMD7 are SD bus mode only: SPI mode has no RCA and selects
 * the card with chip select, so they must not be issued here. */
#define SD_CMD_GO_IDLE_STATE        (0u)
#define SD_CMD_SEND_IF_COND         (8u)
#define SD_CMD_SEND_CSD             (9u)
#define SD_CMD_STOP_TRANSMISSION    (12u)
#define SD_CMD_SET_BLOCKLEN         (16u)
#define SD_CMD_READ_SINGLE_BLOCK    (17u)
#define SD_CMD_READ_MULTIPLE_BLOCK  (18u)
#define SD_CMD_APP_CMD              (55u)
#define SD_CMD_READ_OCR             (58u)
#define SD_CMD_SEND_OP_COND         (41u)

/* R1 status bits */
#define SD_R1_ILLEGAL_COMMAND       (0x04u)

/* HCS bit of the ACMD41 argument, asks for block addressing */
#define SD_OCR_HCS                  (0x40000000u)
/* CCS bit of the OCR, set when the card runs block addressing */
#define SD_OCR_CCS                  (0x40u)

/* Response payload sizes */
#define SD_CSD_LEN                  (16u)
#define SD_OCR_LEN                  (4u)
#define SD_R7_LEN                   (4u)

/* Protocol timing */
#define SD_CMD_TIMEOUT_MS           (500u)   /* One command / response  */
#define SD_READ_TIMEOUT_MS          (2000u)  /* One data block          */
#define SD_INIT_TIMEOUT_MS          (2000u)  /* ACMD41 poll window      */
#define SD_POWER_ON_CLOCKS          (80u)    /* >74 clocks before CMD0  */

/* Response and token markers */
#define SD_R1_IDLE                  (0x01u)
#define SD_DATA_TOKEN_START         (0xFEu)
#define SD_DATA_TOKEN_STOP          (0x0Fu)
#define SD_TOKEN_NONE               (0xFFu)

/* Data start bit for a command frame, the low 6 bits carry the index */
#define SD_CMD_FRAME_FLAG           (0x40u)

static SdCard_Info_t Sd_Card_Info;

/**
 * @name    SdCard_Crc7
 * @brief   Compute the CRC7 that terminates a command frame.
 * @param   pBuf: first five bytes of the frame.
 * @param   Len: number of bytes to cover.
 * @retval  The seven bit checksum, still unshifted.
 */
static uint8_t SdCard_Crc7(const uint8_t *pBuf, uint32_t Len)
{
    uint8_t  crc = 0u;
    uint8_t  data;
    uint8_t  bit;
    uint32_t i;

    for (i = 0u; i < Len; i++)
    {
        data = pBuf[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc <<= 1;
            if (((data & 0x80u) != 0u) ^ ((crc & 0x80u) != 0u))
            {
                crc ^= 0x09u;
            }
            data <<= 1;
        }
    }

    return (uint8_t)(crc & 0x7Fu);
}

/**
 * @name    SdCard_WaitR1
 * @brief   Poll the bus until the card drives its R1 start bit.
 * @param   pR1: captured response byte.
 * @param   TimeoutMs: how long to keep trying.
 * @retval  SD_CARD_OK, or SD_CARD_ERR_TIMEOUT when nothing arrives in time.
 */
static int SdCard_WaitR1(uint8_t *pR1, uint32_t TimeoutMs)
{
    uint32_t start_tick;
    uint8_t  byte = SD_TOKEN_NONE;

    start_tick = Sd_GetTickMs();
    do
    {
        if (Mcal_SdSpi_TransferByte(SD_TOKEN_NONE, &byte) != SD_SPI_OK)
        {
            return SD_CARD_ERR_TIMEOUT;
        }
        if ((Sd_GetTickMs() - start_tick) > TimeoutMs)
        {
            return SD_CARD_ERR_TIMEOUT;
        }
    } while ((byte & 0x80u) != 0u);

    *pR1 = byte;

    return SD_CARD_OK;
}

/**
 * @name    SdCard_SendCmd
 * @brief   Send a six byte command frame and collect the R1 response.
 * @param   Cmd: command index, without the start bits.
 * @param   Arg: 32 bit argument.
 * @param   pR1: captured R1 response.
 * @retval  SD_CARD_OK, or a negative SdCard_Status_t.
 * @note    The caller owns chip select and must hold it low across the frame.
 */
static int SdCard_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t *pR1)
{
    uint8_t frame[6];
    uint8_t i;

    frame[0] = (uint8_t)(SD_CMD_FRAME_FLAG | Cmd);
    frame[1] = (uint8_t)(Arg >> 24);
    frame[2] = (uint8_t)(Arg >> 16);
    frame[3] = (uint8_t)(Arg >> 8);
    frame[4] = (uint8_t)Arg;
    frame[5] = (uint8_t)((SdCard_Crc7(frame, 5u) << 1) | 0x01u);

    /* One idle byte between frames, then the command itself */
    if (Mcal_SdSpi_WriteDummy(1u) != SD_SPI_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    for (i = 0u; i < 6u; i++)
    {
        if (Mcal_SdSpi_TransferByte(frame[i], (uint8_t *)0) != SD_SPI_OK)
        {
            return SD_CARD_ERR_TIMEOUT;
        }
    }

    return SdCard_WaitR1(pR1, SD_CMD_TIMEOUT_MS);
}

/**
 * @name    SdCard_SendCmdApp
 * @brief   Send CMD55 followed by the application command itself.
 * @param   Cmd: application command index.
 * @param   Arg: 32 bit argument for the application command.
 * @param   pR1: captured R1 response of the application command.
 * @retval  SD_CARD_OK, or a negative SdCard_Status_t.
 * @note    The caller owns chip select and must hold it low across both frames.
 */
static int SdCard_SendCmdApp(uint8_t Cmd, uint32_t Arg, uint8_t *pR1)
{
    uint8_t r1;

    if (SdCard_SendCmd(SD_CMD_APP_CMD, 0u, &r1) != SD_CARD_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }
    if ((r1 & SD_R1_ILLEGAL_COMMAND) != 0u)
    {
        return SD_CARD_ERR_RESPONSE;
    }

    return SdCard_SendCmd(Cmd, Arg, pR1);
}

/**
 * @name    SdCard_ReadDataBlock
 * @brief   Wait for the data token and pull in one block.
 * @param   pBuf: destination buffer.
 * @param   Len: bytes to read, normally SD_BLOCK_SIZE.
 * @retval  SD_CARD_OK, or a negative SdCard_Status_t.
 */
static int SdCard_ReadDataBlock(uint8_t *pBuf, uint32_t Len)
{
    uint32_t start_tick;
    uint8_t  token = SD_TOKEN_NONE;

    start_tick = Sd_GetTickMs();
    do
    {
        if (Mcal_SdSpi_TransferByte(SD_TOKEN_NONE, &token) != SD_SPI_OK)
        {
            return SD_CARD_ERR_TIMEOUT;
        }
        if ((Sd_GetTickMs() - start_tick) > SD_READ_TIMEOUT_MS)
        {
            return SD_CARD_ERR_TOKEN;
        }
    } while (token == SD_TOKEN_NONE);

    if (token != SD_DATA_TOKEN_START)
    {
        return SD_CARD_ERR_TOKEN;
    }

    if (Mcal_SdSpi_ReadBytes(pBuf, Len) != SD_SPI_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    /* Trailing CRC16, the card always sends it and SPI mode lets us drop it */
    if (Mcal_SdSpi_WriteDummy(2u) != SD_SPI_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    return SD_CARD_OK;
}

/**
 * @name    SdCard_StopTransmission
 * @brief   End a CMD18 stream with CMD12 and wait for the card to go idle.
 * @param   None
 * @retval  SD_CARD_OK, or a negative SdCard_Status_t.
 * @note    The caller owns chip select and must hold it low across the frame.
 */
static int SdCard_StopTransmission(void)
{
    uint32_t start_tick;
    uint8_t  r1;
    uint8_t  byte;

    if (SdCard_SendCmd(SD_CMD_STOP_TRANSMISSION, 0u, &r1) != SD_CARD_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    /* The card holds MISO low while it finishes the aborted transfer */
    start_tick = Sd_GetTickMs();
    do
    {
        if (Mcal_SdSpi_TransferByte(SD_TOKEN_NONE, &byte) != SD_SPI_OK)
        {
            return SD_CARD_ERR_TIMEOUT;
        }
        if ((Sd_GetTickMs() - start_tick) > SD_READ_TIMEOUT_MS)
        {
            return SD_CARD_ERR_TIMEOUT;
        }
    } while (byte != SD_TOKEN_NONE);

    return SD_CARD_OK;
}

/**
 * @name    SdCard_ParseCsd
 * @brief   Derive card capacity from the 16 byte CSD register.
 * @param   pCsd: raw CSD as returned by CMD9.
 * @param   pInfo: info structure to fill in.
 * @retval  SD_CARD_OK, or SD_CARD_ERR_UNSUPPORTED for an unknown CSD layout.
 */
static int SdCard_ParseCsd(const uint8_t *pCsd, SdCard_Info_t *pInfo)
{
    uint32_t word[3];
    uint32_t c_size;
    uint32_t block_count;
    uint32_t block_len;

    word[0] = ((uint32_t)pCsd[0] << 24) | ((uint32_t)pCsd[1] << 16) |
              ((uint32_t)pCsd[2] << 8)  |  (uint32_t)pCsd[3];
    word[1] = ((uint32_t)pCsd[4] << 24) | ((uint32_t)pCsd[5] << 16) |
              ((uint32_t)pCsd[6] << 8)  |  (uint32_t)pCsd[7];
    word[2] = ((uint32_t)pCsd[8] << 24) | ((uint32_t)pCsd[9] << 16) |
              ((uint32_t)pCsd[10] << 8) |  (uint32_t)pCsd[11];

    if (((word[0] >> 30) & 0x03u) == 1u)
    {
        /* CSD version 2.0, used by SDHC and SDXC, C_SIZE lives at bits 69..48 */
        c_size = ((word[1] & 0x3Fu) << 16) | (word[2] >> 16);

        pInfo->CapacityKb = (c_size + 1u) * 512u;
        pInfo->BlockCount = (c_size + 1u) * 1024u;
        return SD_CARD_OK;
    }

    if (((word[0] >> 30) & 0x03u) == 0u)
    {
        /* CSD version 1.0, standard capacity, byte addressed */
        block_len   = 1u << ((word[1] >> 16) & 0x0Fu);
        c_size      = ((word[1] & 0x3FFu) << 2) | (word[2] >> 30);
        block_count = (c_size + 1u) << (((word[2] >> 15) & 0x07u) + 2u);

        pInfo->CapacityKb = (block_count * block_len) / 1024u;
        pInfo->BlockCount = (block_count * block_len) / SD_BLOCK_SIZE;
        return SD_CARD_OK;
    }

    return SD_CARD_ERR_UNSUPPORTED;
}

/**
 * @name    SdCard_EnterIdleState
 * @brief   Run CMD0 to put the card into SPI mode and the idle state.
 * @param   None
 * @retval  SD_CARD_OK, or a negative SdCard_Status_t.
 */
static int SdCard_EnterIdleState(void)
{
    uint8_t r1 = SD_TOKEN_NONE;

    Mcal_SdSpi_CsLow();
    if (SdCard_SendCmd(SD_CMD_GO_IDLE_STATE, 0u, &r1) != SD_CARD_OK)
    {
        Mcal_SdSpi_CsHigh();
        log_info("[SD] CMD0 silent, check the card seating, wiring and supply\r\n");
        return SD_CARD_ERR_NO_CARD;
    }
    if (r1 != SD_R1_IDLE)
    {
        Mcal_SdSpi_CsHigh();
        log_info("[SD] CMD0 r1=0x%02X, expected 0x01\r\n", r1);
        return SD_CARD_ERR_NO_CARD;
    }

    return SD_CARD_OK;
}

/**
 * @name    SdCard_ProbeVoltage
 * @brief   Run CMD8 to find out whether this is a v2 card.
 * @param   pIsV2: set to 1 for SD v2.x, 0 for SD v1.x.
 * @retval  SD_CARD_OK, or a negative SdCard_Status_t.
 * @note    Chip select must already be low.
 */
static int SdCard_ProbeVoltage(uint8_t *pIsV2)
{
    uint8_t check[SD_R7_LEN];
    uint8_t r1;

    *pIsV2 = 0u;

    if (SdCard_SendCmd(SD_CMD_SEND_IF_COND, 0x1AAu, &r1) != SD_CARD_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    /* A v1 card answers with the illegal command bit set */
    if ((r1 & SD_R1_ILLEGAL_COMMAND) != 0u)
    {
        return SD_CARD_OK;
    }
    if (r1 != SD_R1_IDLE)
    {
        return SD_CARD_ERR_RESPONSE;
    }

    if (Mcal_SdSpi_ReadBytes(check, SD_R7_LEN) != SD_SPI_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    /* Echo of the voltage byte and the 0xAA check pattern */
    if ((check[2] != 0x01u) || (check[3] != 0xAAu))
    {
        return SD_CARD_ERR_VOLTAGE;
    }

    *pIsV2 = 1u;

    return SD_CARD_OK;
}

/**
 * @name    SdCard_PollOcr
 * @brief   Loop ACMD41 until the card leaves the idle state.
 * @param   IsV2: 1 when the card answered CMD8.
 * @retval  SD_CARD_OK, or a negative SdCard_Status_t.
 * @note    Chip select must already be low.
 */
static int SdCard_PollOcr(uint8_t IsV2)
{
    uint32_t start_tick;
    uint32_t arg;
    uint8_t  r1;

    arg        = (IsV2 != 0u) ? SD_OCR_HCS : 0u;
    start_tick = Sd_GetTickMs();

    do
    {
        if (SdCard_SendCmdApp(SD_CMD_SEND_OP_COND, arg, &r1) != SD_CARD_OK)
        {
            return SD_CARD_ERR_TIMEOUT;
        }
        if ((Sd_GetTickMs() - start_tick) > SD_INIT_TIMEOUT_MS)
        {
            return SD_CARD_ERR_TIMEOUT;
        }
    } while (r1 != 0u);

    return SD_CARD_OK;
}

/**
 * @name    SdCard_ReadOcr
 * @brief   Run CMD58 and report whether the card uses block addressing.
 * @param   pIsHighCapacity: set to 1 for SDHC / SDXC.
 * @retval  SD_CARD_OK, or a negative SdCard_Status_t.
 * @note    Chip select must already be low.
 */
static int SdCard_ReadOcr(uint8_t *pIsHighCapacity)
{
    uint8_t ocr[SD_OCR_LEN];
    uint8_t r1;

    if (SdCard_SendCmd(SD_CMD_READ_OCR, 0u, &r1) != SD_CARD_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }
    if (r1 != 0u)
    {
        return SD_CARD_ERR_RESPONSE;
    }

    if (Mcal_SdSpi_ReadBytes(ocr, SD_OCR_LEN) != SD_SPI_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    *pIsHighCapacity = ((ocr[0] & SD_OCR_CCS) != 0u) ? 1u : 0u;

    return SD_CARD_OK;
}

/**
 * @name    App_SdCard_Init
 * @brief   Bring up SPI2 and run the full SD SPI bring-up sequence.
 * @param   None
 * @retval  SD_CARD_OK on success, a negative SdCard_Status_t otherwise.
 */
int App_SdCard_Init(void)
{
    uint8_t  is_v2;
    uint8_t  is_high_capacity;
    uint8_t  r1;
    int      ret;

    Sd_Card_Info.Type        = SD_CARD_TYPE_UNKNOWN;
    Sd_Card_Info.BlockCount  = 0u;
    Sd_Card_Info.CapacityKb  = 0u;

    Mcal_SdSpi_GpioInit();
    Mcal_SdSpi_SetBaudRate(SD_SPI_INIT_PRESCALER);

    /* The card needs at least 74 clocks with chip select released */
    Mcal_SdSpi_CsHigh();
    if (Mcal_SdSpi_WriteDummy(SD_POWER_ON_CLOCKS) != SD_SPI_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    ret = SdCard_EnterIdleState();
    if (ret != SD_CARD_OK)
    {
        return ret;
    }

    ret = SdCard_ProbeVoltage(&is_v2);
    if (ret != SD_CARD_OK)
    {
        Mcal_SdSpi_CsHigh();
        return ret;
    }

    ret = SdCard_PollOcr(is_v2);
    if (ret != SD_CARD_OK)
    {
        Mcal_SdSpi_CsHigh();
        return ret;
    }

    ret = SdCard_ReadOcr(&is_high_capacity);
    if (ret != SD_CARD_OK)
    {
        Mcal_SdSpi_CsHigh();
        return ret;
    }

    Sd_Card_Info.Type = (is_high_capacity != 0u) ? SD_CARD_TYPE_SDHC
                       : ((is_v2 != 0u) ? SD_CARD_TYPE_SD2 : SD_CARD_TYPE_SD1);

    /* SPI mode has no RCA, CMD9 always takes argument 0 */
    if (SdCard_SendCmd(SD_CMD_SEND_CSD, 0u, &r1) != SD_CARD_OK)
    {
        Mcal_SdSpi_CsHigh();
        return SD_CARD_ERR_TIMEOUT;
    }
    if (r1 != 0u)
    {
        Mcal_SdSpi_CsHigh();
        return SD_CARD_ERR_RESPONSE;
    }
    if (SdCard_ReadDataBlock(Sd_Card_Info.Csd, SD_CSD_LEN) != SD_CARD_OK)
    {
        Mcal_SdSpi_CsHigh();
        return SD_CARD_ERR_TIMEOUT;
    }

    ret = SdCard_ParseCsd(Sd_Card_Info.Csd, &Sd_Card_Info);
    if (ret != SD_CARD_OK)
    {
        Mcal_SdSpi_CsHigh();
        return ret;
    }

    /* A block addressed card is fixed at 512 bytes, a byte addressed one is not */
    if (Sd_Card_Info.Type != SD_CARD_TYPE_SDHC)
    {
        if (SdCard_SendCmd(SD_CMD_SET_BLOCKLEN, SD_BLOCK_SIZE, &r1) != SD_CARD_OK)
        {
            Mcal_SdSpi_CsHigh();
            return SD_CARD_ERR_TIMEOUT;
        }
        if (r1 != 0u)
        {
            Mcal_SdSpi_CsHigh();
            return SD_CARD_ERR_RESPONSE;
        }
    }

    Mcal_SdSpi_CsHigh();
    if (Mcal_SdSpi_WriteDummy(1u) != SD_SPI_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    /* Card is talking, move to the data phase clock */
    Mcal_SdSpi_SetBaudRate(SD_SPI_FAST_PRESCALER);

    log_info("[SD] init ok type=%d blocks=%d capacity=%d KB\r\n",
             (int)Sd_Card_Info.Type,
             (int)Sd_Card_Info.BlockCount, (int)Sd_Card_Info.CapacityKb);

    return SD_CARD_OK;
}

/**
 * @name    App_SdCard_GetInfo
 * @brief   Copy out the card description captured during bring-up.
 * @param   pInfo: destination, left untouched when it is NULL.
 * @retval  SD_CARD_OK on success, SD_CARD_ERR_PARAM when pInfo is NULL.
 */
int App_SdCard_GetInfo(SdCard_Info_t *pInfo)
{
    uint8_t i;

    if (pInfo == (SdCard_Info_t *)0)
    {
        return SD_CARD_ERR_PARAM;
    }

    pInfo->Type       = Sd_Card_Info.Type;
    pInfo->BlockCount = Sd_Card_Info.BlockCount;
    pInfo->CapacityKb = Sd_Card_Info.CapacityKb;

    for (i = 0u; i < 16u; i++)
    {
        pInfo->Csd[i] = Sd_Card_Info.Csd[i];
    }

    return SD_CARD_OK;
}

/**
 * @name    App_SdCard_ReadBlocks
 * @brief   Read consecutive 512 byte blocks starting at the given block address.
 * @param   BlockAddr: first block, always a block number not a byte offset.
 * @param   pBuf: destination buffer, must hold BlockCount * 512 bytes.
 * @param   BlockCount: number of blocks to read, must not be zero.
 * @retval  SD_CARD_OK on success, a negative SdCard_Status_t otherwise.
 */
int App_SdCard_ReadBlocks(uint32_t BlockAddr, uint8_t *pBuf, uint32_t BlockCount)
{
    uint32_t arg;
    uint32_t i;
    uint8_t  r1;
    int      ret;

    if ((pBuf == (uint8_t *)0) || (BlockCount == 0u))
    {
        return SD_CARD_ERR_PARAM;
    }
    if ((BlockAddr + BlockCount) > Sd_Card_Info.BlockCount)
    {
        return SD_CARD_ERR_PARAM;
    }

    /* A byte addressed card wants an offset, everything else wants a block index */
    arg = (Sd_Card_Info.Type == SD_CARD_TYPE_SDHC) ? BlockAddr
                                                   : (BlockAddr * SD_BLOCK_SIZE);

    Mcal_SdSpi_CsLow();

    if (BlockCount == 1u)
    {
        ret = SdCard_SendCmd(SD_CMD_READ_SINGLE_BLOCK, arg, &r1);
    }
    else
    {
        ret = SdCard_SendCmd(SD_CMD_READ_MULTIPLE_BLOCK, arg, &r1);
    }

    if (ret != SD_CARD_OK)
    {
        Mcal_SdSpi_CsHigh();
        return ret;
    }
    if (r1 != 0u)
    {
        Mcal_SdSpi_CsHigh();
        return SD_CARD_ERR_RESPONSE;
    }

    for (i = 0u; i < BlockCount; i++)
    {
        ret = SdCard_ReadDataBlock(&pBuf[i * SD_BLOCK_SIZE], SD_BLOCK_SIZE);
        if (ret != SD_CARD_OK)
        {
            Mcal_SdSpi_CsHigh();
            return ret;
        }
    }

    if (BlockCount > 1u)
    {
        ret = SdCard_StopTransmission();
        if (ret != SD_CARD_OK)
        {
            Mcal_SdSpi_CsHigh();
            return ret;
        }
    }

    Mcal_SdSpi_CsHigh();
    if (Mcal_SdSpi_WriteDummy(1u) != SD_SPI_OK)
    {
        return SD_CARD_ERR_TIMEOUT;
    }

    return SD_CARD_OK;
}
