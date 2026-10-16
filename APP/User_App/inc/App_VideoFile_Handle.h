/**
 * @file    App_VideoFile_Handle.h
 * @brief   Locate and verify the video payload stored on the SD card.
 * @author  D0cC
 * @date    2026-09-14
 * @version V1.0.0
 *
 * @note    Application layer. The card carries a raw payload at a fixed
 *          absolute LBA, written by tools/sd_write_video.py. No file system
 *          is involved on this side: a 512 byte header describes the payload
 *          and the payload follows it contiguously. The tool owns the layout,
 *          this module only reads it back.
 */

#ifndef APP_VIDEO_FILE_HANDLE_H
#define APP_VIDEO_FILE_HANDLE_H

#include <stdint.h>

/* Card layout. Must match tools/sd_write_video.py. */
#define VIDEO_HEADER_LBA        (2048u)
#define VIDEO_PAYLOAD_LBA       (2049u)
#define VIDEO_MAGIC_LEN         (8u)
#define VIDEO_LAYOUT_VERSION    (1u)

/* Header field offsets inside the 512 byte header sector */
#define VIDEO_OFF_VERSION       (8u)
#define VIDEO_OFF_BYTES         (12u)
#define VIDEO_OFF_CRC32         (16u)
#define VIDEO_OFF_BLOCKS        (20u)

/**
 * @brief  Return codes.
 */
typedef enum
{
    VIDEO_OK = 0,                   /* Verified, or this chunk went through  */
    VIDEO_BUSY = 1,                 /* Stepwise verify still has work left   */
    VIDEO_ERR_PARAM = -1,           /* Bad argument from the caller          */
    VIDEO_ERR_NO_HEADER = -2,       /* Header sector would not read          */
    VIDEO_ERR_MAGIC = -3,           /* Card was not written by the tool      */
    VIDEO_ERR_VERSION = -4,         /* Layout version this build cannot read */
    VIDEO_ERR_SIZE = -5,            /* Payload does not fit on the card      */
    VIDEO_ERR_READ = -6,            /* Block read failed part way through    */
    VIDEO_ERR_CRC = -7              /* Payload CRC32 disagrees with header   */
} VideoFile_Status_t;

/**
 * @brief  What the header says about the payload.
 */
typedef struct
{
    uint32_t LayoutVersion;         /* Layout revision the card was written with */
    uint32_t PayloadBytes;          /* Payload length, padding excluded          */
    uint32_t PayloadCrc32;          /* zlib CRC32 over the first PayloadBytes    */
    uint32_t BlockCount;            /* Payload rounded up to whole 512 B blocks  */
} VideoFile_Info_t;

/**
 * @name    App_VideoFile_Open
 * @brief   Read the header sector, validate it and arm a verify pass.
 * @param   None
 * @retval  VIDEO_OK, or a negative VideoFile_Status_t.
 * @note    Must run after App_SdCard_Init(). Resets the running CRC32, so a
 *          verify pass always starts from the top of the payload.
 */
int App_VideoFile_Open(void);

/**
 * @name    App_VideoFile_GetInfo
 * @brief   Copy out what the header said.
 * @param   pInfo: destination, left untouched when it is NULL.
 * @retval  VIDEO_OK, or VIDEO_ERR_PARAM when pInfo is NULL.
 */
int App_VideoFile_GetInfo(VideoFile_Info_t *pInfo);

/**
 * @name    App_VideoFile_ReadBytes
 * @brief   Read a run of payload bytes starting at a byte offset.
 * @param   Offset: byte offset into the payload, must be block aligned.
 * @param   pBuf: destination, must hold Len bytes.
 * @param   Len: bytes wanted, must be a multiple of SD_BLOCK_SIZE.
 * @param   pRead: bytes actually read, short when the run reached the end.
 * @retval  VIDEO_OK, or a negative VideoFile_Status_t.
 * @note    The stream is treated as BlockCount whole blocks, so the payload
 *          length is rounded up to a block boundary. The tail bytes are the
 *          zero padding the writer tool added; sending them is harmless and
 *          it keeps every read block aligned.
 */
int App_VideoFile_ReadBytes(uint32_t Offset, uint8_t *pBuf, uint32_t Len,
                            uint32_t *pRead);

/**
 * @name    App_VideoFile_GetStreamBytes
 * @brief   Payload length rounded up to a whole block, the streaming length.
 * @param   None
 * @retval  Byte count, or 0 when no header has been parsed yet.
 */
uint32_t App_VideoFile_GetStreamBytes(void);

/**
 * @name    App_VideoFile_VerifyChunk
 * @brief   Fold one chunk of the payload into the running CRC32.
 * @param   None
 * @retval  VIDEO_BUSY when more payload is left, VIDEO_OK once the whole
 *          payload has been read and its CRC32 matches the header, or a
 *          negative VideoFile_Status_t.
 * @note    One chunk is a fixed number of blocks, small enough to call from a
 *          scheduler slot without starving the rest of the main loop.
 */
int App_VideoFile_VerifyChunk(void);

#endif /* APP_VIDEO_FILE_HANDLE_H */
