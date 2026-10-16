/**
 * @file    Mcal_SensI2C.h
 * @brief   Software (bit-banged) I2C master for the sensor buses.
 * @author  D0cC
 * @date    2026-09-29
 * @version V1.2.0
 *
 * @note    Targets the N32H785 using the N32H7xx standard peripheral library.
 *
 *          This is a GPIO bit-bang master rather than a hardware I2C instance,
 *          for two reasons:
 *
 *            1. AFIO_RMP_CFG carries remap bits for SPI1..SPI4, I2S, SDMMC, ETH
 *               and FEMC, but none for I2C, so every hardware instance is
 *               locked to one pin pair while this board's ports are almost
 *               fully committed to SDRAM, ETH and the two SPI links.
 *            2. The magnetometer may have to move for magnetic reasons, and two
 *               macros are cheaper to move than a fixed alternate function.
 *
 *          Two independent buses, one device each: the sensors are on flying
 *          leads, so a shared pair would put both breakout boards on one stub
 *          and let either one hold the whole bus down.
 *
 *          All four pins are open drain. The MCU's internal pull-ups are left
 *          enabled on purpose: they are far too weak to meet the I2C rise time
 *          on their own, but they hold the bus at a defined idle level and are
 *          enough to limp through bring-up if the external pull-ups are
 *          missing. Fit 4.7k to the sensor supply on EACH bus, for 100 kHz.
 *
 *          Bit timing comes from the DWT cycle counter, NOT from
 *          systick_delay_us() in bsp/delay.c. That function reprograms SysTick
 *          and then disables it, which would stop the 1 ms timebase the
 *          cooperative scheduler and lwIP both run on.
 *
 *          Every public function is documented here; Mcal_SensI2C.c carries
 *          the implementation only.
 */

#ifndef MCAL_SENS_I2C_H
#define MCAL_SENS_I2C_H

#include <stdint.h>
#include "n32h7xx.h"
#include "n32h7xx_rcc.h"
#include "n32h7xx_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 1. Bus pins. These are the only things to change if the sensors are wired
 *    differently on the board.
 *
 *    Both buses sit on GPIOC so one clock gate covers them, on pairs nothing
 *    else in this firmware claims: PC0..PC5 carry ETH and SDRAM chip select /
 *    clock enable, PC15 is taken, and nothing in the tree touches PC6..PC8.
 *    No code writes AFIO->RMP_CFG, so no remap can move a peripheral onto
 *    them, and the SD card runs in SPI mode rather than SDMMC.
 *
 *    Bus 0 has moved twice on this board, both times because the pads could not
 *    carry a bus at all. It started on PC9/PC10: the magnetometer, and then a
 *    working LSM6DSV16X as a control, both answered on PC11/PC12 and neither
 *    answered on PC9/PC10. It moved to PC6/PC7, which is worse -- PC6 reads low
 *    at idle with nothing connected to it, and clearing its analog function
 *    does not change that, so the pad itself is bad. It now sits on PB8/PB9,
 *    the port the SD card has been working on all along.
 *
 *    Mcal_SensI2C_Init() now proves both pads before a bus is used: it pulls
 *    each line down, reads it back, releases it and reads it again. A pair that
 *    fails is reported as a bus that would not come up -- which is the only
 *    warning available before anything is connected to it.
 *
 *    The lesson worth keeping is the ANAEN one. ANAEN marks the pads that
 *    double as analog inputs, and an earlier pair that included analog-capable
 *    pins came up with SCL stuck low and SDA fine, the pad's own pull-up
 *    apparently not reaching it. If a pair misbehaves exactly that way, suspect
 *    it first -- but if clearing the analog bit does not fix it, the pad is bad
 *    and the bus has to move.
 *
 *    CONFIRM AGAINST THE SCHEMATIC. A wrong pair costs nothing in firmware but
 *    everything on the bench. */
#define SENS_I2C0_SCL_PORT              GPIOB
#define SENS_I2C0_SCL_PIN               GPIO_PIN_8
#define SENS_I2C0_SDA_PORT              GPIOB
#define SENS_I2C0_SDA_PIN               GPIO_PIN_9

#define SENS_I2C1_SCL_PORT              GPIOC
#define SENS_I2C1_SCL_PIN               GPIO_PIN_11
#define SENS_I2C1_SDA_PORT              GPIOC
#define SENS_I2C1_SDA_PIN               GPIO_PIN_12

/* Analog function bits to clear for the pads above, per bus, and the register
 * those bits live in.
 *
 *    Some pads double as analog inputs and come up in that mode, where the pad
 *    can neither drive nor release its line. The symptom is exact: SCL reads
 *    low at idle while SDA reads fine, and every transfer times out before the
 *    first address byte. On GPIOC the analog-capable pads are PC0..PC6, PC8 and
 *    PC13; on GPIOB only PB0 and PB1 are. PC11/PC12 and PB8/PB9 are not, which
 *    is why both current pairs are safe.
 *
 *    Both masks are zero for that reason. The mechanism stays because a pair
 *    that does include an analog-capable pad would need it, and because the
 *    failure it prevents is otherwise indistinguishable from a broken wire. */
#define SENS_I2C0_ANAEN_REG             (&AFIO->ANAEN_CFG1)   /* GPIOB */
#define SENS_I2C0_ANAEN_CLR             (0u)
#define SENS_I2C1_ANAEN_REG             (&AFIO->ANAEN_CFG2)   /* GPIOC */
#define SENS_I2C1_ANAEN_CLR             (0u)

#define SENS_I2C0_GPIO_CLK              RCC_AHB5_PERIPHEN_M7_GPIOB
#define SENS_I2C1_GPIO_CLK              RCC_AHB5_PERIPHEN_M7_GPIOC

/* Both clock gates are enabled together so that moving a bus to a different
 * port cannot leave its clock gated and its pins dead */
#define SENS_I2C_GPIO_CLK_ENABLE()                                          \
    do {                                                                    \
        RCC_EnableAHB5PeriphClk1(                                           \
            (uint32_t)(SENS_I2C0_GPIO_CLK | SENS_I2C1_GPIO_CLK), ENABLE);   \
    } while (0)

/* 2. Bus speed.
 *
 *    400 kHz is the Fast-mode ceiling and both parts are specified for it, so
 *    that is the target. What the bus actually runs at sits below the nominal
 *    figure: each bit costs two half-periods plus the GPIO and call overhead
 *    between them, and SCL is never driven high, only released and then waited
 *    on -- so a slow pull-up rise slows the bus down instead of corrupting a
 *    bit. The clock rate this macro sets is a floor, not an exact rate.
 *
 *    SENS_I2C_HALF_PERIOD_US is integer-divided, so only 5, 2 and 1 are
 *    reachable: 100 kHz, 200 kHz and 400 kHz and above. There is no 300 kHz.
 *
 *    4.7k pull-ups to the sensor supply are what 400 kHz assumes. With much
 *    weaker ones the rise time sets the rate, and the first symptom is NACKs
 *    during the init scan -- drop to 200000 if that happens. 100000 is the
 *    speed every stage up to and including the gyroscope was verified at. */
#define SENS_I2C_BUS_HZ                 (400000u)
#define SENS_I2C_HALF_PERIOD_US         (500000u / SENS_I2C_BUS_HZ)

/* 3. Timeouts. A busy-wait budget in loop iterations, enough to ride out any
 *    slave that clock-stretches and still give up on a bus shorted low. */
#define SENS_I2C_POLL_TIMEOUT           (200000u)

/* The largest address a 7-bit slave can have; anything above it is a caller
 * mistake rather than a device that did not answer */
#define SENS_I2C_ADDR7_MAX              (0x7Fu)

/* Address of the 7-bit slave shifted up to leave the R/W bit clear */
#define SENS_I2C_ADDR_W(addr7)          ((uint8_t)(((addr7) << 1) | 0x00u))
#define SENS_I2C_ADDR_R(addr7)          ((uint8_t)(((addr7) << 1) | 0x01u))

/**
 * @brief  The two sensor buses. One device per bus, so no address can collide
 *         and neither bus depends on the other's wiring.
 */
typedef enum
{
    SENS_I2C_BUS_0 = 0,             /* PC6 = SCL, PC7 = SDA: MMC5617WA      */
    SENS_I2C_BUS_1 = 1,             /* PC11 = SCL, PC12 = SDA: LSM6DSV16X   */
    SENS_I2C_BUS_COUNT = 2
} SensI2C_Bus_t;

/**
 * @brief  Driver return codes.
 */
typedef enum
{
    SENS_I2C_OK = 0,                /* Transfer completed                       */
    SENS_I2C_ERR_PARAM = -1,        /* Bad argument from the caller             */
    SENS_I2C_ERR_NACK_ADDR = -2,    /* No slave answered the address byte       */
    SENS_I2C_ERR_NACK_DATA = -3,    /* Slave NAKed a data byte                  */
    SENS_I2C_ERR_BUS_STUCK = -4,    /* SDA held low, bus is not idle            */
    SENS_I2C_ERR_TIMEOUT = -5       /* Slave stretched the clock past the budget*/
} SensI2C_Status_t;

/**
 * @name    Mcal_SensI2C_Init
 * @brief   Configure both pins of one bus as open-drain outputs, release the
 *          bus and start the cycle counter used for bit timing.
 * @param   Bus: which bus to bring up.
 * @retval  SENS_I2C_OK, or SENS_I2C_ERR_BUS_STUCK if SDA is held low at idle,
 *          or SENS_I2C_ERR_PARAM for a bus index out of range.
 * @note    Idempotent for the cycle counter: calling it for the second bus
 *          only reconfigures GPIO.
 */
int32_t Mcal_SensI2C_Init(SensI2C_Bus_t Bus);

/**
 * @name    Mcal_SensI2C_Probe
 * @brief   Send the address byte alone and report whether the slave ACKed.
 * @param   Bus: which bus to use.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @retval  SENS_I2C_OK if the address was acknowledged, a negative
 *          SensI2C_Status_t otherwise.
 */
int32_t Mcal_SensI2C_Probe(SensI2C_Bus_t Bus, uint8_t DevAddr7);

/**
 * @name    Mcal_SensI2C_WriteReg
 * @brief   Write one byte to one register.
 * @param   Bus: which bus to use.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @param   Reg: register address.
 * @param   Value: byte to write.
 * @retval  SENS_I2C_OK on success, a negative SensI2C_Status_t otherwise.
 */
int32_t Mcal_SensI2C_WriteReg(SensI2C_Bus_t Bus, uint8_t DevAddr7, uint8_t Reg, uint8_t Value);

/**
 * @name    Mcal_SensI2C_WriteRegs
 * @brief   Write consecutive bytes starting at one register address.
 * @param   Bus: which bus to use.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @param   Reg: first register address.
 * @param   pBuf: source buffer, must not be NULL and must hold Len bytes.
 * @param   Len: number of bytes, must not be zero.
 * @retval  SENS_I2C_OK on success, a negative SensI2C_Status_t otherwise.
 */
int32_t Mcal_SensI2C_WriteRegs(SensI2C_Bus_t Bus, uint8_t DevAddr7, uint8_t Reg,
                               const uint8_t *pBuf, uint16_t Len);

/**
 * @name    Mcal_SensI2C_ReadReg
 * @brief   Read one byte from one register.
 * @param   Bus: which bus to use.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @param   Reg: register address.
 * @param   pValue: destination, must not be NULL.
 * @retval  SENS_I2C_OK on success, a negative SensI2C_Status_t otherwise.
 */
int32_t Mcal_SensI2C_ReadReg(SensI2C_Bus_t Bus, uint8_t DevAddr7, uint8_t Reg, uint8_t *pValue);

/**
 * @name    Mcal_SensI2C_ReadRegs
 * @brief   Read consecutive bytes starting at one register address, in a
 *          single repeated-start transaction so the slave keeps its
 *          auto-increment pointer.
 * @param   Bus: which bus to use.
 * @param   DevAddr7: 7-bit slave address, not shifted.
 * @param   Reg: first register address.
 * @param   pBuf: destination, must not be NULL and must hold Len bytes.
 * @param   Len: number of bytes, must not be zero.
 * @retval  SENS_I2C_OK on success, a negative SensI2C_Status_t otherwise.
 */
int32_t Mcal_SensI2C_ReadRegs(SensI2C_Bus_t Bus, uint8_t DevAddr7, uint8_t Reg,
                              uint8_t *pBuf, uint16_t Len);

/**
 * @name    Mcal_SensI2C_Scan
 * @brief   Probe every address in the 7-bit range and record the ones that
 *          answer. Bring-up aid: this is what turns "no data" into either
 *          "the part is not there" or "the part is there and the driver is
 *          wrong".
 * @param   Bus: which bus to scan.
 * @param   pFound: destination array for the addresses that ACKed, or NULL
 *                  to only count them.
 * @param   MaxFound: capacity of pFound.
 * @retval  Number of addresses found, or a negative SensI2C_Status_t when the
 *          bus itself is stuck.
 */
int32_t Mcal_SensI2C_Scan(SensI2C_Bus_t Bus, uint8_t *pFound, uint8_t MaxFound);

/**
 * @name    Mcal_SensI2C_GetLineLevels
 * @brief   Release both lines and read back their idle levels.
 * @param   Bus: which bus to look at.
 * @param   pSclHigh: set to 1 when SCL is high at idle, 0 when it is held low.
 * @param   pSdaHigh: the same for SDA.
 * @retval  SENS_I2C_OK, or SENS_I2C_ERR_PARAM for a bad bus index or a NULL
 *          destination.
 * @note    Bring-up aid. A line that reads low at idle with nothing driving it
 *          is the difference between "the slave did not answer" and "this wire
 *          never worked", and it is the one thing the transfer code cannot
 *          report because by then the transfer has already failed.
 */
int32_t Mcal_SensI2C_GetLineLevels(SensI2C_Bus_t Bus, uint8_t *pSclHigh, uint8_t *pSdaHigh);

/**
 * @name    Mcal_SensI2C_BusRecover
 * @brief   Clock a slave out of a half-finished transfer: up to nine pulses
 *          on SCL, then a stop condition. Call it after a NACK or timeout
 *          before retrying, otherwise one stuck slave wedges its bus for
 *          every device on it.
 * @param   Bus: which bus to recover.
 * @retval  SENS_I2C_OK once the bus is idle, SENS_I2C_ERR_BUS_STUCK if SDA
 *          never came back high.
 */
int32_t Mcal_SensI2C_BusRecover(SensI2C_Bus_t Bus);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_SENS_I2C_H */
