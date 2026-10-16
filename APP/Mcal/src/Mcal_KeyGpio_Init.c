/**
 * @file    Mcal_KeyGpio_Init.c
 * @brief   Key GPIO driver implementation: the calibration button on PC0.
 * @author  D0cC
 * @date    2026-09-29
 * @version V1.0.0
 *
 * @note    Targets the N32H785 using the N32H7xx standard peripheral library.
 */

#include "Mcal_KeyGpio_Init.h"

/**
 * @name    Mcal_KeyGpio_Init
 * @brief   Configure the key pin as an input with its pull resistor enabled.
 * @param   None
 * @retval  None
 * @note    The pull is not optional. A tact switch can only ever pull the pad
 *          to ground, so without the pull-up a released key floats and reads
 *          whatever the neighbouring traces are doing.
 * @note    Nothing is logged here; this layer does no printing. The pad level
 *          at this moment is reported by App_Sensor_Init().
 */
void Mcal_KeyGpio_Init(void)
{
    GPIO_InitType gpio_init;

    KEY_GPIO_CLK_ENABLE();

    GPIO_InitStruct(&gpio_init);
    gpio_init.Pin       = KEY_CALIB_PIN;
    gpio_init.GPIO_Mode = KEY_GPIO_MODE;
    gpio_init.GPIO_Pull = KEY_GPIO_PULL;
    GPIO_InitPeripheral(KEY_CALIB_PORT, &gpio_init);
}

/**
 * @name    Key_GetLevel
 * @brief   Read the raw electrical level on the key pad.
 * @param   None
 * @retval  0 or 1, with no assumption about which of them means pressed.
 * @note    Reading an input-mode pin returns the pad, which is the point: this
 *          is the same measurement App_Sensor_Init() logs to prove the pad is
 *          alive.
 */
uint8_t Key_GetLevel(void)
{
    return (uint8_t)GPIO_ReadInputDataBit(KEY_CALIB_PORT, KEY_CALIB_PIN);
}

/**
 * @name    Key_IsPressed
 * @brief   Whether the key is currently held down.
 * @param   None
 * @retval  1 while held, 0 while released.
 * @note    No debounce timer. The caller polls this from the scheduler's 10 ms
 *          slot and edge-detects the result, so a bounce can at worst produce
 *          one spurious edge inside a single poll period, and the calibration
 *          state machine tolerates a repeated start by ignoring it.
 */
uint8_t Key_IsPressed(void)
{
    return (Key_GetLevel() == KEY_PRESSED_LEVEL) ? 1u : 0u;
}
