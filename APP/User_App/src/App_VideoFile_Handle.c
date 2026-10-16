/**
 * @file    App_VideoFile_Handle.c
 * @brief   Locate and verify the video payload stored on the SD card.
 * @author  D0cC
 * @date    2026-09-14
 * @version V1.0.0
 *
 * @note    Targets the N32H785 dev board, M7 core, standard peripheral library.
 *          Everything below the block level lives in App_SdCard_Handle.c.
 *          The payload layout is written by tools/sd_write_video.py and is
 *          documented in that tool; this file is the read back side of it.
 */

#include "App_VideoFile_Handle.h"
#include "App_SdCard_Handle.h"
#include "App_Crc32.h"
#include "log.h"

#define VIDEO_CHUNK_BLOCKS      (16u)


static const uint8_t Video_Magic[VIDEO_MAGIC_LEN] =
{
    (uint8_t)'N', (uint8_t)'3', (uint8_t)'2', (uint8_t)'V',
    (uint8_t)'I', (uint8_t)'D', (uint8_t)'0', (uint8_t)'1'
};

static VideoFile_Info_t Video_Info;

/* Running verify state, reset by App_VideoFile_Open */
static uint32_t Video_Crc;
static uint32_t Video_Done;

/**
 * @name    VideoFile_GetU32
 * @brief   Pull a little endian 32 bit field out of the header.
 * @param   pBuf: header buffer.
 * @param   Offset: byte offset of the field.
 * @retval  The field value.
 * @note    Read by offset on purpose: the header buffer must never be cast
 *          to a struct, alignment and padding are not guaranteed.
 */
static uint32_t VideoFile_GetU32(const uint8_t *pBuf, uint32_t Offset)
{
    return ((uint32_t)pBuf[Offset])
         | ((uint32_t)pBuf[Offset + 1u] << 8)
         | ((uint32_t)pBuf[Offset + 2u] << 16)
         | ((uint32_t)pBuf[Offset + 3u] << 24);
}

/**
 * @name    App_VideoFile_Open
 * @brief   Read the header sector, validate it and arm a verify pass.
 * @param   None
 * @retval  VIDEO_OK, or a negative VideoFile_Status_t.
 */
int App_VideoFile_Open(void)
{
    static uint8_t Header[SD_BLOCK_SIZE];
    SdCard_Info_t  card;
    uint32_t       last_lba;
    uint8_t        i;

    Video_Info.LayoutVersion = 0u;
    Video_Info.PayloadBytes  = 0u;
    Video_Info.PayloadCrc32  = 0u;
    Video_Info.BlockCount    = 0u;

    Video_Crc  = 0xFFFFFFFFu;
    Video_Done = 0u;

    if (App_SdCard_ReadBlocks(VIDEO_HEADER_LBA, Header, 1u) != SD_CARD_OK)
    {
        return VIDEO_ERR_NO_HEADER;
    }

    for (i = 0u; i < VIDEO_MAGIC_LEN; i++)
    {
        if (Header[i] != Video_Magic[i])
        {
            return VIDEO_ERR_MAGIC;
        }
    }

    Video_Info.LayoutVersion = VideoFile_GetU32(Header, VIDEO_OFF_VERSION);
    Video_Info.PayloadBytes  = VideoFile_GetU32(Header, VIDEO_OFF_BYTES);
    Video_Info.PayloadCrc32  = VideoFile_GetU32(Header, VIDEO_OFF_CRC32);
    Video_Info.BlockCount    = VideoFile_GetU32(Header, VIDEO_OFF_BLOCKS);

    if (Video_Info.LayoutVersion != VIDEO_LAYOUT_VERSION)
    {
        return VIDEO_ERR_VERSION;
    }
    if ((Video_Info.PayloadBytes == 0u) || (Video_Info.BlockCount == 0u))
    {
        return VIDEO_ERR_MAGIC;
    }

    /* The whole payload has to sit inside the card */
    if (App_SdCard_GetInfo(&card) != SD_CARD_OK)
    {
        return VIDEO_ERR_SIZE;
    }
    last_lba = VIDEO_PAYLOAD_LBA + Video_Info.BlockCount;
    if (last_lba > card.BlockCount)
    {
        return VIDEO_ERR_SIZE;
    }

    log_info("[VID] bytes=%d blocks=%d crc=%08x\r\n",
             (int)Video_Info.PayloadBytes, (int)Video_Info.BlockCount,
             (unsigned int)Video_Info.PayloadCrc32);

    return VIDEO_OK;
}

/**
 * @name    App_VideoFile_GetInfo
 * @brief   Copy out what the header said.
 * @param   pInfo: destination, left untouched when it is NULL.
 * @retval  VIDEO_OK, or VIDEO_ERR_PARAM when pInfo is NULL.
 */
int App_VideoFile_GetInfo(VideoFile_Info_t *pInfo)
{
    if (pInfo == (VideoFile_Info_t *)0)
    {
        return VIDEO_ERR_PARAM;
    }

    pInfo->LayoutVersion = Video_Info.LayoutVersion;
    pInfo->PayloadBytes  = Video_Info.PayloadBytes;
    pInfo->PayloadCrc32  = Video_Info.PayloadCrc32;
    pInfo->BlockCount    = Video_Info.BlockCount;

    return VIDEO_OK;
}

/**
 * @name    App_VideoFile_ReadBytes
 * @brief   Read a run of payload bytes starting at a byte offset.
 * @param   Offset: byte offset into the payload, must be block aligned.
 * @param   pBuf: destination, must hold Len bytes.
 * @param   Len: bytes wanted, must be a multiple of SD_BLOCK_SIZE.
 * @param   pRead: bytes actually read, short when the run reached the end.
 * @retval  VIDEO_OK, or a negative VideoFile_Status_t.
 */
int App_VideoFile_ReadBytes(uint32_t Offset, uint8_t *pBuf, uint32_t Len,
                            uint32_t *pRead)
{
    uint32_t stream_bytes;
    uint32_t left;
    uint32_t blocks;

    if ((pBuf == (uint8_t *)0) || (pRead == (uint32_t *)0))
    {
        return VIDEO_ERR_PARAM;
    }
    if (((Offset % SD_BLOCK_SIZE) != 0u) || ((Len % SD_BLOCK_SIZE) != 0u))
    {
        return VIDEO_ERR_PARAM;
    }

    *pRead = 0u;

    stream_bytes = App_VideoFile_GetStreamBytes();
    if ((stream_bytes == 0u) || (Offset >= stream_bytes))
    {
        return VIDEO_OK;
    }

    left = stream_bytes - Offset;
    if (Len > left)
    {
        Len = left;
    }

    blocks = Len / SD_BLOCK_SIZE;
    if (blocks == 0u)
    {
        return VIDEO_OK;
    }

    if (App_SdCard_ReadBlocks(VIDEO_PAYLOAD_LBA + (Offset / SD_BLOCK_SIZE),
                              pBuf, blocks) != SD_CARD_OK)
    {
        return VIDEO_ERR_READ;
    }

    *pRead = blocks * SD_BLOCK_SIZE;

    return VIDEO_OK;
}

/**
 * @name    App_VideoFile_GetStreamBytes
 * @brief   Payload length rounded up to a whole block, the streaming length.
 * @param   None
 * @retval  Byte count, or 0 when no header has been parsed yet.
 */
uint32_t App_VideoFile_GetStreamBytes(void)
{
    return Video_Info.BlockCount * SD_BLOCK_SIZE;
}

/**
 * @name    App_VideoFile_VerifyChunk
 * @brief   Fold one chunk of the payload into the running CRC32.
 * @param   None
 * @retval  VIDEO_BUSY when payload is left, VIDEO_OK when the whole payload
 *          read back and matched, or a negative VideoFile_Status_t.
 */
int App_VideoFile_VerifyChunk(void)
{
    static uint8_t Buf[VIDEO_CHUNK_BLOCKS * SD_BLOCK_SIZE];
    uint32_t       left;
    uint32_t       blocks;
    uint32_t       take;

    if (Video_Info.PayloadBytes == 0u)
    {
        return VIDEO_ERR_MAGIC;
    }
    if (Video_Done >= Video_Info.PayloadBytes)
    {
        return VIDEO_OK;
    }

    left   = Video_Info.PayloadBytes - Video_Done;
    blocks = (left + SD_BLOCK_SIZE - 1u) / SD_BLOCK_SIZE;
    if (blocks > VIDEO_CHUNK_BLOCKS)
    {
        blocks = VIDEO_CHUNK_BLOCKS;
    }

    /* Video_Done is always block aligned here, the tail chunk ends the pass */
    if (App_SdCard_ReadBlocks(VIDEO_PAYLOAD_LBA + (Video_Done / SD_BLOCK_SIZE),
                              Buf, blocks) != SD_CARD_OK)
    {
        return VIDEO_ERR_READ;
    }

    take = blocks * SD_BLOCK_SIZE;
    if (take > left)
    {
        take = left;
    }

    Video_Crc  = App_Crc32_Update(Buf, take, Video_Crc);
    Video_Done += take;

    if (Video_Done < Video_Info.PayloadBytes)
    {
        return VIDEO_BUSY;
    }

    Video_Crc ^= 0xFFFFFFFFu;

    log_info("[VID] read %d bytes, crc=%08x expect=%08x\r\n",
             (int)Video_Done, (unsigned int)Video_Crc,
             (unsigned int)Video_Info.PayloadCrc32);

    return (Video_Crc == Video_Info.PayloadCrc32) ? VIDEO_OK : VIDEO_ERR_CRC;
}
