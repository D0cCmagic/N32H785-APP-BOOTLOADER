#ifndef __MCAL_UART_H__
#define __MCAL_UART_H__

/**
 * @file    Mcal_Uart.h
 * @brief   USART1 byte pipe to the Linux board: DMA receive and DMA transmit.
 *
 * @note    This file moves bytes and nothing else. It has no idea that the
 *          bytes are remote-control frames: it does not look for a header, a
 *          length or a checksum, and it does not care what a frame means. The
 *          protocol lives in the application layer, which is the only place
 *          that knows the wire format.
 *
 * @note    The calls below are the same whichever receive path is built --
 *          UART_LINK_RX_USE_DMA in Mcal_Uart_Cfg.h picks between a polled
 *          receiver and a DMA one driven by the USART idle interrupt. Transmit
 *          is polled in both. The reasoning for each is in Mcal_Uart.c.
 *
 * @note    Board wiring, link rate, channel numbers and buffer sizes are in
 *          Mcal_Uart_Cfg.h. Nothing here has a number in it.
 */

#include <stdint.h>
#include "Mcal_Uart_Cfg.h"      /* the mode switch and the buffer sizes */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Mcal_Uart_Init
 * @brief   Bring USART1 up on the receive path the configuration selects.
 * @param   None
 * @retval  None
 * @note    Both paths are idempotent: a second call does nothing.
 */
void Mcal_Uart_Init(void);

/**
 * @name    Mcal_Uart_Send
 * @brief   Write bytes out of the data register, waiting for room.
 * @param   Data: bytes to send.
 * @param   Len: how many, 1 to UART_LINK_TX_BUF_SIZE.
 * @retval  0 on success, -1 if the transmitter never made room or the length is
 *          unusable.
 * @note    Bounded throughout: a stopped transmitter cannot freeze the caller.
 */
int32_t Mcal_Uart_Send(uint8_t *Data, uint32_t Len);

/**
 * @name    Mcal_Uart_TxWaitDone
 * @brief   Block until the transmit DMA has finished, or the bound expires.
 * @param   SpinLimit: poll iterations before giving up.
 * @retval  1 if the transmitter is idle, 0 if the bound ran out.
 * @note    The ONLY blocking call in this module, for exactly one caller: the
 *          OTA path, which reboots the part and must let the last message out.
 */
uint8_t Mcal_Uart_TxWaitDone(uint32_t SpinLimit);

/* ---- The received burst, and the one flag that goes with it ---------------
 *
 * The receiver puts a whole burst in the buffer, records how long it was, and
 * then raises the flag. The application reads the buffer directly. There is no
 * take-it-out call, because there is nothing left to protect against: the driver
 * does not touch the buffer again until the flag has been cleared, and clearing
 * the flag is what consuming the burst means.
 *
 * The contract is three steps:
 *
 *     if (Mcal_Uart_RxFlag != 0u)             // is there a burst?
 *     {
 *         Mcal_Uart_RxFlag = 0u;              // consume it
 *         ... read Mcal_Uart_RxBuf[0 .. Mcal_Uart_RxLen - 1] ...
 *     }
 *
 * The flag is the only thing crossing between the driver and the application,
 * and it is there because the two are asynchronous: a burst ends when the
 * hardware says so, which is not when the application happens to come round.
 * Without it the application could not tell a new burst from the one it has
 * already handled.
 */
extern volatile uint8_t  Mcal_Uart_RxFlag;   /* 1 = a whole burst is waiting */
extern volatile uint16_t Mcal_Uart_RxLen;    /* its length in bytes */
extern const uint8_t    *Mcal_Uart_RxBuf;    /* where those bytes are */
extern volatile uint32_t Mcal_Uart_RxDrop;   /* bursts thrown away; see below */

#if (UART_LINK_RX_USE_DMA == 0)
/**
 * @name    Mcal_Uart_Poll
 * @brief   Polled receive mode only: take whatever the receiver has.
 * @param   None
 * @retval  None
 * @note    Only declared when the polled path is the one being built. In DMA mode
 *          the function does not exist at all -- the channel and the idle
 *          interrupt have already collected the bytes -- which is why callers
 *          guard the call with UART_LINK_RX_USE_DMA rather than relying on a
 *          no-op.
 */
void Mcal_Uart_Poll(void);
#endif


#ifdef __cplusplus
}
#endif


#endif /* __MCAL_UART_H__ */
