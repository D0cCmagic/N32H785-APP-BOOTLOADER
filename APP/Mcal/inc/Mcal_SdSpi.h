/**
 * @file    Mcal_SdSpi.h
 * @brief   SPI2 bring-up and byte level primitives for the SD card socket.
 * @author  D0cC
 * @date    2026-09-11
 * @version V1.1.0
 *
 * @note    Hardware layer only. Nothing here knows what an SD command is:
 *          the card protocol is built on top of these calls in
 *          App_SdCard_Handle.c. Transfers are polled; the DMA path that the
 *          streaming phase needs would be added here, behind
 *          Mcal_SdSpi_ReadBytes.
 */

#ifndef MCAL_SD_SPI_H
#define MCAL_SD_SPI_H

#include <stdint.h>
#include "n32h7xx.h"
#include "n32h7xx_gpio.h"
#include "n32h7xx_spi.h"
#include "n32h7xx_rcc.h"
#include "Mcal_SdSpi_Cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 1 ms system timebase, defined in main.c and driven by SysTick */
uint32_t sys_now(void);
#define SdSpi_GetTickMs()  sys_now()

/**
 * @brief  Return codes of the byte level primitives.
 */
typedef enum
{
    SD_SPI_OK = 0,              /* Transfer completed                     */
    SD_SPI_ERR_TIMEOUT = -1     /* The SPI never raised the expected flag */
} SdSpi_Status_t;

/**
 * @name    Mcal_SdSpi_GpioInit
 * @brief   Configure the SPI2 pins and drive chip select high.
 * @param   None
 * @retval  None
 */
void Mcal_SdSpi_GpioInit(void);

/**
 * @name    Mcal_SdSpi_SetBaudRate
 * @brief   Bring up SPI2, or change its clock divider while running.
 * @param   Prescaler: one of the SPI_BR_PRESCALER_* values.
 * @retval  None
 */
void Mcal_SdSpi_SetBaudRate(uint16_t Prescaler);

/**
 * @name    Mcal_SdSpi_CsHigh
 * @brief   Deselect the card.
 * @param   None
 * @retval  None
 */
void Mcal_SdSpi_CsHigh(void);

/**
 * @name    Mcal_SdSpi_CsLow
 * @brief   Select the card.
 * @param   None
 * @retval  None
 */
void Mcal_SdSpi_CsLow(void);

/**
 * @name    Mcal_SdSpi_TransferByte
 * @brief   Exchange one byte with the card, polled.
 * @param   Tx: byte to shift out.
 * @param   pRx: captured byte, may be NULL when the answer is not wanted.
 * @retval  SD_SPI_OK, or SD_SPI_ERR_TIMEOUT when the SPI never flags.
 */
int Mcal_SdSpi_TransferByte(uint8_t Tx, uint8_t *pRx);

/**
 * @name    Mcal_SdSpi_WriteDummy
 * @brief   Clock the bus with 0xFF and discard whatever comes back.
 * @param   Count: number of bytes to clock out.
 * @retval  SD_SPI_OK, or SD_SPI_ERR_TIMEOUT.
 */
int Mcal_SdSpi_WriteDummy(uint32_t Count);

/**
 * @name    Mcal_SdSpi_ReadBytes
 * @brief   Clock out 0xFF and capture what the card sends back.
 * @param   pBuf: destination buffer.
 * @param   Len: number of bytes to read.
 * @retval  SD_SPI_OK, or SD_SPI_ERR_TIMEOUT.
 */
int Mcal_SdSpi_ReadBytes(uint8_t *pBuf, uint32_t Len);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_SD_SPI_H */
