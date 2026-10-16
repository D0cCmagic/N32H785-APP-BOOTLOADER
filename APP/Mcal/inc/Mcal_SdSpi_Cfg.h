/**
 * @file    Mcal_SdSpi_Cfg.h
 * @brief   SPI2 pin, clock and DMA configuration for the SD card socket.
 * @author  D0cC
 * @date    2026-09-11
 * @version V1.1.0
 *
 * @note    Targets the N32H785 dev board, M7 core, standard peripheral library.
 *          The card runs on SPI2; SPI1 is already taken by the R900PNR module.
 *          SPI2 sits on APB2, which RCC_SetSysClkToMode0 leaves at 150 MHz.
 *          This header is hardware only. The SD protocol constants live in
 *          App_SdCard_Handle.h.
 */

#ifndef MCAL_SD_SPI_CFG_H
#define MCAL_SD_SPI_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* 1. SPI peripheral. Both SPI1 and SPI2 live on APB2 and share one clock
 *    enable function, but they have separate enable bits. */
#define SD_SPIx                         SPI2
#define SD_SPIx_CLK                     RCC_APB2_PERIPHEN_M7_SPI2
#define SD_SPIx_APBx_CLK_CMD            RCC_EnableAPB2PeriphClk2
#define SD_SPIx_IRQn                    SPI2_IRQn

/* 2. SPI2 pins on GPIOB: PB12=CS, PB13=SCK, PB14=MISO, PB15=MOSI.
 *    This is the default SPI2 pin set, so AFIO_RMP_CFG.SPI2SEL must stay 0.
 *
 *    Board note: on the N32H785XIB7_STB, PB14 and PB15 also reach the USB
 *    Type-C connector through the 0 ohm resistors R123 / R130, and PB12 has
 *    R148 left unpopulated. With no cable plugged in the stubs are harmless,
 *    but if the SPI link misbehaves, move the card to the alternative set
 *    listed below and re-check. */
#define SD_SPI_SCK_PIN                  GPIO_PIN_13
#define SD_SPI_SCK_PORT                 GPIOB
#define SD_SPI_SCK_AF                   GPIO_AF4          /* SPI2_SCK */

#define SD_SPI_MISO_PIN                 GPIO_PIN_14
#define SD_SPI_MISO_PORT                GPIOB
#define SD_SPI_MISO_AF                  GPIO_AF4          /* SPI2_MISO */

#define SD_SPI_MOSI_PIN                 GPIO_PIN_15
#define SD_SPI_MOSI_PORT                GPIOB
#define SD_SPI_MOSI_AF                  GPIO_AF4          /* SPI2_MOSI */

/* Chip select is driven in software; any free GPIO works */
#define SD_SPI_CS_PIN                   GPIO_PIN_12
#define SD_SPI_CS_PORT                  GPIOB

/* Alternative SPI2 pin set, avoids the USB Type-C routing above.
 * Swap the four groups above for these if the default set will not run.
 *   SD_SPI_SCK_PIN   PB10  AF4
 *   SD_SPI_MISO_PIN  PC2   AF6
 *   SD_SPI_MOSI_PIN  PC1   AF4
 *   SD_SPI_CS_PIN    PB12  (plain GPIO output, remap setting irrelevant)
 */
#define SD_ALT_SPI_SCK_PIN              GPIO_PIN_10
#define SD_ALT_SPI_MISO_PIN             GPIO_PIN_2
#define SD_ALT_SPI_MOSI_PIN             GPIO_PIN_1
#define SD_ALT_SPI_SCK_AF               GPIO_AF4
#define SD_ALT_SPI_MISO_AF              GPIO_AF6
#define SD_ALT_SPI_MOSI_AF              GPIO_AF4

/* One clock enable covers every pin, they are all on GPIOB */
#define SD_GPIO_CLK_ENABLE()                                            \
    do {                                                                \
        RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB, ENABLE);   \
    } while (0)

/* 3. SPI parameters for SD SPI mode: Mode 0, 8 bit, MSB first, software NSS.
 *    CRCPoly is unused, SD appends its own CRC7 / CRC16 bytes. */
#define SD_SPI_MODE                     SPI_MODE_MASTER
#define SD_SPI_DIR                      SPI_DIR_DOUBLELINE_FULLDUPLEX
#define SD_SPI_DATA_SIZE                SPI_DATA_SIZE_8BITS
#define SD_SPI_CPOL                     SPI_CLKPOL_LOW
#define SD_SPI_CPHA                     SPI_CLKPHA_FIRST_EDGE
#define SD_SPI_NSS                      SPI_NSS_SOFT
#define SD_SPI_FIRST_BIT                SPI_FB_MSB
#define SD_SPI_CRC_POLY                 7

/* APB2 is 150 MHz, so the prescaler sets the card clock directly.
 * Init must stay in the 100..400 kHz band, /256 gives 586 kHz, the slowest
 * the divider chain offers and low enough for every card seen so far. */
#define SD_SPI_INIT_PRESCALER           SPI_BR_PRESCALER_256   /* 586 kHz  */
/* Data phase. Measured 2026-09-14 on this board's wiring: every prescaler
 * down to /16 reads block 0 with a valid 0x55 0xAA signature, while /8
 * (18.75 MHz) loses the data token and returns 0x00 instead of 0xFE.
 * The MISO line PB14 also reaches the USB Type-C connector through R123,
 * which costs the margin at the top of the range. Keep /16 until that
 * signal integrity is sorted out. */
#define SD_SPI_FAST_PRESCALER           SPI_BR_PRESCALER_16    /* 9.38 MHz */

/* 4. Byte transfer. The SPI hardware is clocked by the card, so a missing
 *    flag means the bus is stuck, not that the card is slow. */
#define SD_SPI_DUMMY_BYTE               (0xFFu)  /* MISO idles high        */
#define SD_SPI_XFER_TIMEOUT_MS          (500u)   /* One byte in either way */

/* 5. DMA setup for the streaming phase, not yet used.
 *    DMA1 CH0 / CH1 belong to the R900PNR on SPI1, so the card takes CH2/CH3.
 *      DMA1 CH2 = SPI2 TX (M2P), DMAMUX1 CH2 -> SPI2_TX
 *      DMA1 CH3 = SPI2 RX (P2M), DMAMUX1 CH3 -> SPI2_RX
 *    Enabling this needs the DMA_ChHwHsIfType value for the SPI2 handshaking
 *    interface, which is not confirmed yet. See the note in Mcal_SdSpi.c. */
#define SD_DMAx                         DMA1
#define SD_DMAx_CLK                     RCC_AHB1_PERIPHEN_M7_DMA1
#define SD_DMAMUX_ID                    DMAMUX1_ID
#define SD_DMAMUXx_CLK                  RCC_AHB1_PERIPHEN_M7_DMAMUX1

#define SD_DMA_TX_CHANNEL               DMA_CHANNEL_2
#define SD_DMA_TX_IRQn                  DMA1_Channel2_IRQn
#define SD_DMA_TX_DMAMUX_CH             DMAMUX_CHANNEL_2
#define SD_DMA_TX_DMAMUX_REQ            DMAMUX1_REQUEST_SPI2_TX

#define SD_DMA_RX_CHANNEL               DMA_CHANNEL_3
#define SD_DMA_RX_IRQn                  DMA1_Channel3_IRQn
#define SD_DMA_RX_DMAMUX_CH             DMAMUX_CHANNEL_3
#define SD_DMA_RX_DMAMUX_REQ            DMAMUX1_REQUEST_SPI2_RX

/* SPI data register, the fixed end of every DMA transfer */
#define SD_SPI_DR_ADDR                  ((uint32_t)((uint32_t)SD_SPIx + 0x0CU))

/* DMA writes through the D-Cache, so any receive buffer must be cache line
 * aligned and be invalidated around the transfer. */
#define SD_DMA_ALIGN                    (32u)

#ifdef __cplusplus
}
#endif

#endif /* MCAL_SD_SPI_CFG_H */
