#include "Mcal_InitSum.h"
#include "Mcal_TimerTask.h"
#include "Mcal_Uart.h"
#include "App_Fusion_Handle.h"
#include "log.h"

/**
 * @name    Mcal_NvicClearAll
 * @brief   Disable and unpend every NVIC interrupt line.
 * @param   None
 * @retval  None
 * @note    The NVIC enable bits are CPU state and survive the jump from the
 *          bootloader into the application. The bootloader uses DMA1 channel 0
 *          for its xmodem receive and leaves that line enabled; whatever else
 *          it touched is left behind too. The startup file vectors every one of
 *          those slots at Default_Handler, which is an endless loop, so an
 *          unhandled leftover interrupt hangs the whole firmware the first time
 *          its condition occurs.
 * @note    Clearing everything here, before any module enables its own line,
 *          means the running set is exactly what this application asked for.
 *          SysTick is a core exception (SHCSR), not an NVIC line, so it is not
 *          affected.
 */
static void Mcal_NvicClearAll(void)
{
    uint32_t i;

    for (i = 0u; i < 8u; i++)
    {
        /* ICER: writing 1 disables that interrupt line. ICPR: writing 1
         * clears a pending one (the bootloader may have left those set too). */
        *(volatile uint32_t *)(0xE000E180u + (i * 4u)) = 0xFFFFFFFFu;
        *(volatile uint32_t *)(0xE000E280u + (i * 4u)) = 0xFFFFFFFFu;
    }
    __DSB();
}

/**
 * @name    Mcal_InitSum
 * @brief   Bring up every MCAL module in the required order.
 * @param   None
 * @retval  None
 */
void Mcal_InitSum(void)
{
    int sens_ret;

    /* Start from a clean interrupt set; see the note above. */
    Mcal_NvicClearAll();

    Mcal_TimerTask_Init();

    /* The console comes up first, and before anything that might want to
     * report something. USART1 is the link to the Linux board as well as the
     * console, and it has one owner -- Mcal_Uart -- so this is what puts a
     * working transmitter behind every log line that follows. It is placed
     * after Mcal_TimerTask_Init() only because it does not depend on the
     * timebase; nothing before it prints. */
    Mcal_Uart_Init();

    Mcal_LedGpio_Init();
    log_init();

    /* App_Sensor_Init logs its own bus scan; this only reports the verdict. */
    sens_ret = App_Sensor_Init();
    if (sens_ret != 0)
    {
        log_info("Sensor Init fail, ret = %d \r\n", sens_ret);
    }

    /* The filter is a consumer of the sensors, not another device on the bus:
     * it opens its startup window here and is then filled from the base tick,
     * so the reference it starts from is measured under the same conditions as
     * everything after it. Non-blocking. */
    App_Fusion_Init();

    /* Last, because the base tick is what runs the sampling: every part it
     * reads, and the filter that consumes the sample, has to be configured
     * before the first interrupt. This is the only point in the order that is
     * after all of them. */
    Mcal_SampleTick_Init();
}
