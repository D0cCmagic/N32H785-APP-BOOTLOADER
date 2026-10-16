#include "App_Crc32.h"

/* zlib CRC32: poly 0xEDB88320 reflected, init 0xFFFFFFFF, xorout 0xFFFFFFFF.
 * Bit at a time on purpose - it runs at about 500 KB/s over a 4 Mbit/s stream,
 * roughly 1% of a 600 MHz M7, so a lookup table would buy nothing worth its
 * 1 KB of RAM. Checked against zlib.crc32() on seven vectors including the
 * standard "123456789" -> CBF43926. */
#define CRC32_POLY      (0xEDB88320u)
#define CRC32_INIT      (0xFFFFFFFFu)
#define CRC32_XOROUT    (0xFFFFFFFFu)

/**
 * @name    App_Crc32_Update
 * @brief   Fold one more chunk into a running CRC32.
 * @param   pBuf: data to fold in, must hold Len bytes.
 * @param   Len: number of bytes.
 * @param   Crc: running value, start from 0xFFFFFFFF.
 * @retval  Updated running value.
 */
uint32_t App_Crc32_Update(const uint8_t *pBuf, uint32_t Len, uint32_t Crc)
{
    uint32_t crc = Crc;
    uint32_t i;
    uint8_t  bit;

    for (i = 0u; i < Len; i++)
    {
        crc ^= (uint32_t)pBuf[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1) ^ CRC32_POLY;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
 * @name    App_Crc32
 * @brief   zlib CRC32 over one buffer, start to finish.
 * @param   pBuf: data to fold in, must hold Len bytes.
 * @param   Len: number of bytes.
 * @retval  CRC32 of the buffer.
 */
uint32_t App_Crc32(const uint8_t *pBuf, uint32_t Len)
{
    return App_Crc32_Update(pBuf, Len, CRC32_INIT) ^ CRC32_XOROUT;
}
