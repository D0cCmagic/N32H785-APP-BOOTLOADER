/**
 * @file    Mcal_SdSpi.c
 * @brief   SPI2 bring-up and byte level primitives for the SD card socket.
 * @author  D0cC
 * @date    2026-09-11
 * @version V1.1.0
 *
 * @note    Targets the N32H785 dev board, M7 core, standard peripheral library.
 *          The card is driven on SPI2, SPI1 belongs to the R900PNR module.
 *          Transfers are polled on purpose for this first step: the DMA
 *          handshaking interface number for SPI2 is not confirmed in the
 *          reference manual, and the card hardware is not on the bench yet,
 *          so a DMA path could not be validated. Mcal_SdSpi_ReadBytes is the
 *          seam DMA would replace, the SD state machine above it is untouched
 *          by that change. The SD_DMA_* macros in the config header are in
 *          place.
 */

#include "Mcal_SdSpi.h"

/**
 * @name    Mcal_SdSpi_GpioInit
 * @brief   Configure the SPI2 pins and the software chip select.
 * @param   None
 * @retval  None
 */
void Mcal_SdSpi_GpioInit(void)
{
    GPIO_InitType gpio_init;

    SD_GPIO_CLK_ENABLE();
    GPIO_InitStruct(&gpio_init);

    gpio_init.Pin             = SD_SPI_SCK_PIN;
    gpio_init.GPIO_Mode       = GPIO_MODE_AF_PP;
    gpio_init.GPIO_Pull       = GPIO_NO_PULL;
    gpio_init.GPIO_Alternate  = SD_SPI_SCK_AF;
    GPIO_InitPeripheral(SD_SPI_SCK_PORT, &gpio_init);

    gpio_init.Pin             = SD_SPI_MISO_PIN;
    gpio_init.GPIO_Mode       = GPIO_MODE_AF_PP;
    gpio_init.GPIO_Pull       = GPIO_NO_PULL;
    gpio_init.GPIO_Alternate  = SD_SPI_MISO_AF;
    GPIO_InitPeripheral(SD_SPI_MISO_PORT, &gpio_init);

    gpio_init.Pin             = SD_SPI_MOSI_PIN;
    gpio_init.GPIO_Mode       = GPIO_MODE_AF_PP;
    gpio_init.GPIO_Pull       = GPIO_NO_PULL;
    gpio_init.GPIO_Alternate  = SD_SPI_MOSI_AF;
    GPIO_InitPeripheral(SD_SPI_MOSI_PORT, &gpio_init);

    /* Chip select idles high so the card stays deselected */
    gpio_init.Pin             = SD_SPI_CS_PIN;
    gpio_init.GPIO_Mode       = GPIO_MODE_OUTPUT_PP;
    gpio_init.GPIO_Pull       = GPIO_NO_PULL;
    gpio_init.GPIO_Alternate  = GPIO_AF0;
    GPIO_InitPeripheral(SD_SPI_CS_PORT, &gpio_init);
    Mcal_SdSpi_CsHigh();
}

/**
 * @name    Mcal_SdSpi_SetBaudRate
 * @brief   Bring up SPI2, or change its clock divider while running.
 * @param   Prescaler: one of the SPI_BR_PRESCALER_* values.
 * @retval  None
 */
void Mcal_SdSpi_SetBaudRate(uint16_t Prescaler)
{
    SPI_InitType spi_init;

    SD_SPIx_APBx_CLK_CMD(SD_SPIx_CLK, ENABLE);
    SPI_Enable(SD_SPIx, DISABLE);
    SPI_I2S_DeInit(SD_SPIx);

    SPI_InitStruct(&spi_init);
    spi_init.SpiMode       = SD_SPI_MODE;
    spi_init.DataDirection = SD_SPI_DIR;
    spi_init.DataLen       = SD_SPI_DATA_SIZE;
    spi_init.CLKPOL        = SD_SPI_CPOL;
    spi_init.CLKPHA        = SD_SPI_CPHA;
    spi_init.NSS           = SD_SPI_NSS;
    spi_init.BaudRatePres  = Prescaler;
    spi_init.FirstBit      = SD_SPI_FIRST_BIT;
    spi_init.CRCPoly       = SD_SPI_CRC_POLY;

    SPI_Init(SD_SPIx, &spi_init);
    SPI_Enable(SD_SPIx, ENABLE);
}

/**
 * @name    Mcal_SdSpi_CsHigh
 * @brief   Deselect the card.
 * @param   None
 * @retval  None
 */
void Mcal_SdSpi_CsHigh(void)
{
    GPIO_SetBits(SD_SPI_CS_PORT, SD_SPI_CS_PIN);
}

/**
 * @name    Mcal_SdSpi_CsLow
 * @brief   Select the card.
 * @param   None
 * @retval  None
 */
void Mcal_SdSpi_CsLow(void)
{
    GPIO_ResetBits(SD_SPI_CS_PORT, SD_SPI_CS_PIN);
}

/**
 * @name    Mcal_SdSpi_TransferByte
 * @brief   Exchange one byte with the card, polled.
 * @param   Tx: byte to shift out.
 * @param   pRx: captured byte, may be NULL when the answer is not wanted.
 * @retval  SD_SPI_OK, or SD_SPI_ERR_TIMEOUT when the SPI never flags.
 */
int Mcal_SdSpi_TransferByte(uint8_t Tx, uint8_t *pRx)
{
    uint32_t start_tick;

    start_tick = SdSpi_GetTickMs();
    while (SPI_I2S_GetStatus(SD_SPIx, SPI_I2S_TE_FLAG) == RESET)
    {
        if ((SdSpi_GetTickMs() - start_tick) > SD_SPI_XFER_TIMEOUT_MS)
        {
            return SD_SPI_ERR_TIMEOUT;
        }
    }
    SPI_I2S_TransmitData(SD_SPIx, (uint16_t)Tx);

    start_tick = SdSpi_GetTickMs();
    while (SPI_I2S_GetStatus(SD_SPIx, SPI_I2S_RNE_FLAG) == RESET)
    {
        if ((SdSpi_GetTickMs() - start_tick) > SD_SPI_XFER_TIMEOUT_MS)
        {
            return SD_SPI_ERR_TIMEOUT;
        }
    }

    if (pRx != (uint8_t *)0)
    {
        *pRx = (uint8_t)SPI_I2S_ReceiveData(SD_SPIx);
    }
    else
    {
        (void)SPI_I2S_ReceiveData(SD_SPIx);
    }

    return SD_SPI_OK;
}

/**
 * @name    Mcal_SdSpi_WriteDummy
 * @brief   Clock the bus with 0xFF and discard whatever comes back.
 * @param   Count: number of bytes to clock out.
 * @retval  SD_SPI_OK, or SD_SPI_ERR_TIMEOUT.
 */
int Mcal_SdSpi_WriteDummy(uint32_t Count)
{
    while (Count > 0u)
    {
        if (Mcal_SdSpi_TransferByte(SD_SPI_DUMMY_BYTE, (uint8_t *)0) != SD_SPI_OK)
        {
            return SD_SPI_ERR_TIMEOUT;
        }
        Count--;
    }

    return SD_SPI_OK;
}

/**
 * @name    Mcal_SdSpi_ReadBytes
 * @brief   Clock out 0xFF and capture what the card sends back.
 * @param   pBuf: destination buffer.
 * @param   Len: number of bytes to read.
 * @retval  SD_SPI_OK, or SD_SPI_ERR_TIMEOUT.
 * @note    This is the call a DMA receive path would replace.
 */
int Mcal_SdSpi_ReadBytes(uint8_t *pBuf, uint32_t Len)
{
    uint32_t i;

    for (i = 0u; i < Len; i++)
    {
        if (Mcal_SdSpi_TransferByte(SD_SPI_DUMMY_BYTE, &pBuf[i]) != SD_SPI_OK)
        {
            return SD_SPI_ERR_TIMEOUT;
        }
    }

    return SD_SPI_OK;
}
