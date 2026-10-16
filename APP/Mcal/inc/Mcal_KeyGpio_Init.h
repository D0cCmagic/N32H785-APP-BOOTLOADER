/**
 * @file    Mcal_KeyGpio_Init.h
 * @brief   Key GPIO driver interface: the calibration button on PC0.
 * @author  D0cC
 * @date    2026-09-29
 * @version V1.0.0
 *
 * @note    Targets the N32H785 using the N32H7xx standard peripheral library.
 *
 *          The board carries three user keys, on PC0, PC1 and PC2. This driver
 *          reads one of them; the other two are unused and available.
 *
 *          PC0 was chosen over PC1 because bsp_eth.c binds PC1 to ETH1_MDC in
 *          all three of its pin tables (RMII, MII and GMII). That file is not
 *          in the Keil project today and no Ethernet pin is configured at
 *          run time, so PC1 is a poor choice rather than an occupied one --
 *          but it becomes a real collision the moment Ethernet is compiled.
 *          PC0 carries no such claim.
 *
 *          One caveat this driver cannot design around: PC0, PC1 and PC2 are
 *          all listed in AFIO_ANAEN_CFG2 (bits 0, 1 and 2), the analog-enable
 *          register. Mcal_SensI2C.h picks its pins specifically to stay off
 *          that list, after an earlier bus came up with one pad unable to
 *          release high; the key cannot, because that is where the button is.
 *          The hypothesis was never confirmed -- the LED pads PA2/PA3 are on
 *          the same kind of list and work -- so the pin is used, and the pad's
 *          idle level is logged at start-up by App_Sensor_Init() so that a dead
 *          pad names itself instead of appearing as a button that never
 *          responds.
 */

#ifndef MCAL_KEY_GPIO_INIT_H
#define MCAL_KEY_GPIO_INIT_H

#include <stdint.h>
#include "n32h7xx.h"
#include "n32h7xx_gpio.h"
#include "n32h7xx_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KEY_CALIB_PORT          GPIOC
#define KEY_CALIB_PIN           GPIO_PIN_0

/* Spelled out for the log line that reports the pad's idle level, matching the
 * way the sensors log their part names. Keep it in step with the pin above:
 * nothing checks that it still names the right pad. */
#define KEY_CALIB_NAME          "PC0"

/* All three keys are on GPIOC, so one clock gate covers them all */
#define KEY_GPIO_CLK_ENABLE()                                           \
    do {                                                                \
        RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOC, ENABLE);   \
    } while (0)

#define KEY_GPIO_MODE           GPIO_MODE_INPUT
#define KEY_GPIO_PULL           GPIO_PULL_UP

/* A tact switch wired to ground reads low while it is held down. This is the
 * one thing about the key that the driver assumes rather than measures, and it
 * is why App_Sensor_Init() logs the released level: a board wired the other way
 * shows up there as an idle level of 0, before the first press is ever missed. */
#define KEY_PRESSED_LEVEL       (0u)

void    Mcal_KeyGpio_Init(void);

uint8_t Key_GetLevel(void);
uint8_t Key_IsPressed(void);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_KEY_GPIO_INIT_H */
