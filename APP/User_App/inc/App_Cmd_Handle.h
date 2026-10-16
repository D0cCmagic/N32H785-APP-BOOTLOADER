#ifndef __APP_CMD_HANDLE_H__
#define __APP_CMD_HANDLE_H__

/**
 * @file    App_Cmd_Handle.h
 * @brief   Frame layer: a byte stream in, whole protocol frames out.
 *
 * @note    This layer knows only the frame geometry -- header, length, tail,
 *          checksum. It never sees the bytes of a received frame: the receiver
 *          marks the end of a burst with the idle interrupt, and a burst is one
 *          frame, so the command layer reads and checks that frame where the
 *          driver left it. What is left here is the two things both ends have to
 *          agree on exactly: the checksum, and the shape of a frame on the way
 *          out.
 *
 * @note    Frame layout (handover doc section 2.1), little-endian fields:
 *
 *            offset 0   HEAD   2   AA 55, not covered by the CRC
 *            offset 2   CMD    1
 *            offset 3   LEN    2   DATA length
 *            offset 5   DATA   N
 *            offset 5+N CRC16  2   over CMD + LEN + DATA, low byte first
 *            offset 7+N TAIL   2   0D 0A
 *
 *          Total length = 9 + LEN, from 9 up to 1033 bytes.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CMD_HEAD0        (0xAAu)
#define APP_CMD_HEAD1        (0x55u)
#define APP_CMD_TAIL0        (0x0Du)
#define APP_CMD_TAIL1        (0x0Au)
#define APP_CMD_OVERHEAD     (9u)       /* head + cmd + len + crc + tail */
#define APP_CMD_MAX_DATA     (1024u)
#define APP_CMD_MAX_FRAME    (APP_CMD_OVERHEAD + APP_CMD_MAX_DATA)   /* 1033 */

/**
 * @name    App_Cmd_SendFrame
 * @brief   Build one frame around the given payload and send it.
 * @param   Cmd: the CMD byte (ACK/ERR/CODE bits already set).
 * @param   Data: payload bytes, may be null when Len is 0.
 * @param   Len: payload length, 0 to APP_CMD_MAX_DATA.
 * @retval  0 when the frame went out, -1 when the payload is too long or the
 *          transmitter never made room.
 * @note    Fills in HEAD, LEN, CRC and TAIL. The payload is used as given, so
 *          the sequence byte (laid out in DATA[0]) is the caller's business.
 */
int32_t App_Cmd_SendFrame(uint8_t Cmd, const uint8_t *Data, uint16_t Len);

/**
 * @name    App_Cmd_Crc16
 * @brief   CRC-16/MODBUS over the bytes given.
 * @param   pData: bytes.
 * @param   Len: how many.
 * @retval  The checksum.
 * @note    Poly 0x8005 (reflected 0xA001), init 0xFFFF, reflected in and out,
 *          no final xor. Exposed for the layer above and for tests.
 */
uint16_t App_Cmd_Crc16(const uint8_t *pData, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CMD_HANDLE_H__ */
