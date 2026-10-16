/**
*     Copyright (c) 2025, Nations Technologies Inc.
*
*     All rights reserved.
*
*     This software is the exclusive property of Nations Technologies Inc.
*/

/**
*\*\file main.c
*\*\brief  N32H785 boat controller: MCAL bring-up and the main loop.
*\*\        The radio module is gone from this board. The remote control link
*\*\        arrives over USART1 from the Linux board, which passes the frames
*\*\        through untouched.
*/

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "App_OTA_Handle.h"    
#include "App_VectorTable.h"
#include "Mcal_InitSum.h"
#include "App_TaskServer_Handle.h"
#include "Mcal_Uart.h"

volatile uint32_t TimeBase = 0;
/**
 * @name    sys_now
 * @brief   Return the 1ms system timebase.
 * @param   None
 * @retval  Milliseconds elapsed since TimeBase was reset.
 */
uint32_t sys_now(void)
{
    return TimeBase;
    
}

/**
 * @name    MPU_Config
 * @brief   Configure the MPU with the privileged default memory map.
 * @param   None
 * @retval  None
 */
static void MPU_Config(void)
{
    MPU_Disable();

    MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}




/**
 * @name    main
 * @brief   Bring up clocks, caches and peripherals, then run the main loop.
 * @param   None
 * @retval  Never returns.
 */
int main(void)
{
	
    App_VectorTableInit();   /* Point VTOR at the ITCM image and remap the SysTick slot */
    RCC_ClocksTypeDef clk;
    SCB_InvalidateDCache();
    SCB_InvalidateICache();
    RCC_SetSysClkToMode0();
    MPU_Config();
    SCB_EnableICache();
    SCB_EnableDCache();
    RCC_GetClocksFreqValue(&clk);
    SysTick_CLKSourceConfig(SysTick_CLKSource_CORECLK);

    /* Set the priority grouping before anything sets a priority, and do not
     * inherit it.
     *
     * PRIGROUP decides how the four priority bits are split between pre-emption
     * and sub-priority, and it is one register shared by the whole core. Nothing
     * in this image used to write it, so the value the bootloader left behind
     * applied instead -- Group 2, where only the top two bits pre-empt. Under
     * that grouping the priorities set below (SysTick 1, USART1 2, BTIM1 5) all
     * land in the same pre-emption group, so none of them can interrupt any
     * other, and a long UART handler delays the 1 ms tick and the sampling
     * interrupt for its whole duration.
     *
     * Group 4 gives all four bits to pre-emption, which is what those priority
     * numbers were chosen for: SysTick(1) > USART1(2) > BTIM1(5), strictly
     * ordered. It has to happen before SysTick_Config(), which sets a priority
     * itself. */
    NVIC_SetPriorityGrouping(4u);

    SysTick_Config(clk.M7ClkFreq / 1000);
    NVIC_SetPriority(SysTick_IRQn, 1);
		
	/* Each module logs its own bring-up: the LEDs, the console, the sensor
	 * bus scan, the filter's window and the [TICK] rate line. There is no
	 * role banner here, because there is no longer a role to choose -- the
	 * radio is off this board and what it used to carry now arrives on
	 * USART1 from the Linux side. */
	Mcal_InitSum();
	
	/* After the MCAL has brought the timebase up and cleared the task table */
	App_Task_Init();
	
    while (1)
    {
			Task_Handle_Cycle();         
    }
}
