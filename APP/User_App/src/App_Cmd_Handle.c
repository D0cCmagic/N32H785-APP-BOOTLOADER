/**
 * @file    App_Cmd_Handle.c
 * @brief   Frame geometry: the checksum and the framing, nothing else.
 *
 * @note    There is no parser here and no working buffer. A burst from the link
 *          is one whole frame -- the receiver marks the end of it with the idle
 *          interrupt -- so the command layer reads that frame where the driver
 *          put it and checks it in place. What is left for this file is the two
 *          things that have to agree byte for byte between the two ends: the
 *          checksum, and the shape of a frame when one is sent.
 */

#include "App_Cmd_Handle.h"
#include "Mcal_Uart.h"

/* Frame being sent, kept off the stack: a maximum frame is 1033 bytes. */
static uint8_t App_Cmd_TxBuf[APP_CMD_MAX_FRAME];

/**
 * @name    App_Cmd_Crc16
 * @brief   CRC-16/MODBUS.
 * @param   pData: bytes.
 * @param   Len: how many.
 * @retval  The checksum.
 * @note    Poly 0x8005 (reflected 0xA001), init 0xFFFF, reflected in and out, no
 *          final xor. Over CMD + LEN + DATA -- not the header, not itself, not
 *          the tail.
 */
uint16_t App_Cmd_Crc16(const uint8_t *pData, uint16_t Len)
{
    uint16_t Crc = 0xFFFFu;
    uint16_t i;
    uint8_t  Bit;

    for (i = 0u; i < Len; i++)
    {
        Crc ^= (uint16_t)pData[i];
        for (Bit = 0u; Bit < 8u; Bit++)
        {
            if ((Crc & 0x0001u) != 0u)
            {
                Crc = (uint16_t)((Crc >> 1) ^ 0xA001u);
            }
            else
            {
                Crc = (uint16_t)(Crc >> 1);
            }
        }
    }

    return Crc;
}

/**
 * @name    App_Cmd_SendFrame
 * @brief   Build one frame around the given payload and send it.
 * @param   Cmd: the CMD byte (ACK/ERR/CODE bits already set).
 * @param   Data: payload bytes, may be null when Len is 0.
 * @param   Len: payload length, 0 to APP_CMD_MAX_DATA.
 * @retval  0 when the frame went out, -1 when the payload is too long or the
 *          transmitter never made room.
 * @note    Fills in HEAD, LEN, CRC and TAIL. The payload is used as given, so
 *          the sequence byte -- DATA[0] -- is the caller's business.
 */
int32_t App_Cmd_SendFrame(uint8_t Cmd, const uint8_t *Data, uint16_t Len)
{
    uint16_t Total;
    uint16_t Crc;
    uint16_t i;

    if ((Len > APP_CMD_MAX_DATA) || ((Data == (const uint8_t *)0) && (Len != 0u)))
    {
        return -1;
    }

    App_Cmd_TxBuf[0] = APP_CMD_HEAD0;
    App_Cmd_TxBuf[1] = APP_CMD_HEAD1;
    App_Cmd_TxBuf[2] = Cmd;
    App_Cmd_TxBuf[3] = (uint8_t)(Len & 0xFFu);
    App_Cmd_TxBuf[4] = (uint8_t)((Len >> 8u) & 0xFFu);

    for (i = 0u; i < Len; i++)
    {
        App_Cmd_TxBuf[5u + i] = Data[i];
    }

    Total = (uint16_t)(APP_CMD_OVERHEAD + Len);

    Crc = App_Cmd_Crc16(&App_Cmd_TxBuf[2], (uint16_t)(3u + Len));
    App_Cmd_TxBuf[Total - 4u] = (uint8_t)(Crc & 0xFFu);
    App_Cmd_TxBuf[Total - 3u] = (uint8_t)((Crc >> 8u) & 0xFFu);
    App_Cmd_TxBuf[Total - 2u] = APP_CMD_TAIL0;
    App_Cmd_TxBuf[Total - 1u] = APP_CMD_TAIL1;

    return Mcal_Uart_Send(App_Cmd_TxBuf, (uint32_t)Total);
}
