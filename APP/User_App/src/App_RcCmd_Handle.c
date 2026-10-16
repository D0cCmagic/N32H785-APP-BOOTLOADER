/**
 * @file    App_RcCmd_Handle.c
 * @brief   Command layer: a whole frame in, the answer (if any) out.
 */

#include "App_RcCmd_Handle.h"
#include "App_Cmd_Handle.h"
#include "App_OTA_Handle.h"     /* the byte matcher, fed on the same bytes */
#include "Mcal_Uart.h"          /* the link's hand-off buffer */

/* Largest payload this layer ever builds. M1 answers are one or two bytes;
 * anything the protocol adds later is still far short of this. */
#define APP_RCCMD_REPLY_MAX     (64u)

static uint8_t App_RcCmd_ReplyBuf[APP_RCCMD_REPLY_MAX];

/**
 * @name    App_RcCmd_Reply
 * @brief   Build and send an answer, mirroring the request's sequence byte.
 * @param   Cmd: the CMD byte to send (ACK/ERR/CODE already set).
 * @param   Seq: the sequence byte taken from the request.
 * @param   HasSeq: 1 when the request carried one.
 * @param   Pay: payload after the sequence byte, may be null.
 * @param   PayLen: its length.
 * @retval  None
 */
static void App_RcCmd_Reply(uint8_t Cmd, uint8_t Seq, uint8_t HasSeq,
                            const uint8_t *Pay, uint16_t PayLen)
{
    uint16_t Len = 0u;
    uint16_t i;

    if (HasSeq != 0u)
    {
        App_RcCmd_ReplyBuf[Len] = Seq;
        Len++;
    }

    for (i = 0u; (i < PayLen) && (Len < APP_RCCMD_REPLY_MAX); i++)
    {
        App_RcCmd_ReplyBuf[Len] = Pay[i];
        Len++;
    }

    (void)App_Cmd_SendFrame(Cmd, App_RcCmd_ReplyBuf, Len);
}

void App_RcCmd_Handle(const uint8_t *Frame, uint16_t FrameLen)
{
    uint8_t  Cmd;
    uint8_t  Code;
    uint8_t  Seq = 0u;
    uint8_t  HasSeq;
    uint16_t DataLen;
    const uint8_t *Data;
    uint8_t  Err;

    if ((Frame == (const uint8_t *)0) || (FrameLen < APP_CMD_OVERHEAD))
    {
        return;
    }

    Cmd     = Frame[2];
    DataLen = (uint16_t)Frame[3] | ((uint16_t)Frame[4] << 8u);
    Data    = &Frame[5];

    Code   = (uint8_t)(Cmd & 0x3Fu);
    HasSeq = (DataLen >= 1u) ? 1u : 0u;
    if (HasSeq != 0u)
    {
        Seq = Data[0];
    }

    /* An answer is not answered: that would talk to itself forever. */
    if ((Cmd & 0x80u) != 0u)
    {
        return;
    }

    switch (Code)
    {
        case APP_RCCMD_HEARTBEAT:
            /* Answer with ACK set, same code, no payload. */
            App_RcCmd_Reply((uint8_t)(0x80u | APP_RCCMD_HEARTBEAT), Seq, HasSeq,
                            (const uint8_t *)0, 0u);
            break;

        default:
            /* Not a command this firmware knows: ACK|ERR, code 0, and the
             * body is the error code alone. */
            Err = APP_RCCMD_ERR_UNSUPPORT;
            App_RcCmd_Reply(0xC0u, Seq, HasSeq, &Err, 1u);
            break;
    }
}

/**
 * @name    app_uart_handler
 * @brief   One burst from the link: check its shape, then act on it.
 * @param   None
 * @retval  None
 * @note    The receiver has already put a whole burst in the driver's buffer and
 *          raised the flag. The far end sends one frame per burst, so a burst is
 *          one frame and there is nothing to collect, nothing to accumulate and
 *          nothing to re-synchronise: this reads that frame where it lies,
 *          checks it, and answers it.
 * @note    Frame layout, all little-endian fields:
 *
 *            [0..1]   HEAD  AA 55
 *            [2]      CMD
 *            [3..4]   LEN          = length of DATA
 *            [5]      DATA[0]      = sequence number
 *            [6..]    DATA[1..]    = payload
 *            [5+LEN]  CRC16        over CMD + LEN + DATA
 *            [7+LEN]  TAIL  0D 0A
 *
 *          Total = 9 + LEN, so LEN is what says where the tail and the checksum
 *          are. That is what makes a variable-length frame checkable.
 * @note    The flag is cleared the moment the burst has been classified -- as a
 *          frame or as something else -- and before any of the frame's contents
 *          are checked. That is deliberate and it matters: the driver refuses to
 *          deliver a new burst while the flag is up, so a path that returned
 *          early while leaving the flag set would stop the receiver dead, with
 *          the flag still reading "something is waiting". A frame that fails a
 *          check is consumed and dropped like any other; the next burst is
 *          unaffected.
 * @note    Nothing here blocks except the answer itself, which is the reply
 *          path doing its job.
 */
void app_uart_handler(void)
{
    const uint8_t *Buf = Mcal_Uart_RxBuf;
    uint16_t       i;
    uint16_t       DataLen;
    uint16_t       Total;
    uint16_t       CrcRx;

#if (UART_LINK_RX_USE_DMA == 0)
    /* Polled receive mode has no interrupt to mark the end of a frame, so the
     * bytes have to be collected here first. In DMA mode this call does not
     * exist -- the channel and the idle interrupt have already done it. */
    Mcal_Uart_Poll();
#endif

    if (Mcal_Uart_RxFlag == 0u)
    {
        return;
    }

    /* ---- Two kinds of burst arrive on this port. The header tells them apart,
     * and this is the only branch that decides which one it is. ---- */
    if ((Buf[0] != APP_CMD_HEAD0) || (Buf[1] != APP_CMD_HEAD1))
    {
        /* Not a frame: it is the plain ASCII word "UPDATE", which arms the
         * bootloader. Fed every byte, because the matcher keeps its own state
         * and the word may be split across two bursts.
         *
         * A frame is never offered to the matcher: a payload that happened to
         * contain the word would otherwise be swallowed here and the frame
         * itself never answered. */
        for (i = 0u; i < Mcal_Uart_RxLen; i++)
        {
            if (ota_cmd_feed(Buf[i]) != 0u)
            {
                ota_request_flag = 1u;
                break;
            }
        }

        Mcal_Uart_RxFlag = 0u;          /* 消费掉这一批 */
        return;
    }

    /* ---- It is a frame. Consume it first, before checking any of it: every
     * check below can return, and a return that left the flag up would block
     * the receiver until the end of time. ---- */
    Mcal_Uart_RxFlag = 0u;

    DataLen = (uint16_t)Buf[3] | ((uint16_t)Buf[4] << 8u);
    Total   = (uint16_t)(APP_CMD_OVERHEAD + DataLen);

    /* ---- the shape of the frame ---- */
    if (DataLen > APP_CMD_MAX_DATA)
    {
        return;
    }
    /* The burst has to be exactly this one frame: shorter means it was cut off,
     * longer means what arrived was not a single frame. */
    if ((uint16_t)Mcal_Uart_RxLen != Total)
    {
        return;
    }
    if ((Buf[Total - 2u] != APP_CMD_TAIL0) || (Buf[Total - 1u] != APP_CMD_TAIL1))
    {
        return;
    }

    /* ---- the checksum, over CMD + LEN + DATA ---- */
    CrcRx = (uint16_t)Buf[Total - 4u] | ((uint16_t)Buf[Total - 3u] << 8u);
    if (CrcRx != App_Cmd_Crc16(&Buf[2], (uint16_t)(3u + DataLen)))
    {
        return;
    }

    /* ---- a whole, clean frame ---- */
    App_RcCmd_Handle(Buf, Total);
}
