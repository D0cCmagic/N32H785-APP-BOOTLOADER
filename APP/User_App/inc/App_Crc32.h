#ifndef __APP_CRC32_H__
#define __APP_CRC32_H__

#include <stdint.h>

/**
 * @name    App_Crc32
 * @brief   zlib CRC32 over one buffer, start to finish.
 * @param   pBuf: data to fold in, must hold Len bytes.
 * @param   Len: number of bytes.
 * @retval  CRC32 of the buffer, the value Python's zlib.crc32() produces.
 * @note    This is the value the SD card writer tool stores and the value the
 *          video frames carry, so a mismatch means the bytes differ and not
 *          that two implementations disagree.
 */
uint32_t App_Crc32(const uint8_t *pBuf, uint32_t Len);

/**
 * @name    App_Crc32_Update
 * @brief   Fold one more chunk into a running CRC32.
 * @param   pBuf: data to fold in, must hold Len bytes.
 * @param   Len: number of bytes.
 * @param   Crc: running value, start from 0xFFFFFFFF.
 * @retval  Updated running value. Invert it with ^ 0xFFFFFFFF only once, after
 *          the last chunk.
 * @note    Split out so a payload can be checksummed a piece at a time without
 *          holding all of it in RAM. App_Crc32 is the one-shot form.
 */
uint32_t App_Crc32_Update(const uint8_t *pBuf, uint32_t Len, uint32_t Crc);

#endif
