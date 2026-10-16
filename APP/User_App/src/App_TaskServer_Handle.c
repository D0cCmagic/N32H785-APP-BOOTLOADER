/**
 * @file    App_TaskServer_Handle.c
 * @brief   Cooperative scheduler task slots called from the main loop.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.1.0
 *
 * @note    None
 */

#include "App_TaskServer_Handle.h"
#include "Mcal_InitSum.h"
#include "Mcal_TimerTask.h"
#include "App_Sensor_Handle.h"
#include "App_Fusion_Handle.h"
#include "App_FusionLog_Handle.h"
#include "App_OTA_Handle.h"
#include "App_RcCmd_Handle.h"



void Task_10ms_Handle(void)
{
    app_uart_handler();         /* 链路收数 → 喂 OTA → 帧层组帧并应答 */
    App_Sensor_Poll();
    //App_Fusion_Log();
}

void App_Sensor_Tick(void)
{
    if (App_Sensor_UpdateData_Tick() != 0)
    {
        App_Fusion_Update();
    }
}

/**
 * @name    Task_20ms_Handle
 * @brief   Poll for an OTA upgrade request.
 * @param   None
 * @retval  None
 */
void Task_20ms_Handle(void)
{
	OTA_UpdateCheck_Handle();
}

/**
 * @name    Task_50ms_Handle
 * @brief   Empty slot; the sensor poll moved to the 10ms slot.
 * @param   None
 * @retval  None
 */
void Task_50ms_Handle(void)
{
    /* Kept registered so the 50ms period stays available */
}

/**
 * @name    Task_200ms_Handle
 * @brief   Blink LED1.
 * @param   None
 * @retval  None
 */
void Task_200ms_Handle(void)
{
    Led_Toggle(LED_ID_1);   /* PA2 */
}

/**
 * @name    Task_300ms_Handle
 * @brief   Blink LED2.
 * @param   None
 * @retval  None
 */
void Task_300ms_Handle(void)
{
    Led_Toggle(LED_ID_2);   /* PA3 */
}

/**
 * @name    App_Task_Init
 * @brief   Register every periodic slot with the scheduler.
 * @param   None
 * @retval  None
 * @note    Called from main() after Mcal_InitSum(), and not from the MCAL's own
 *          init: which work runs on which period is the application's business,
 *          and registering it there is what made the scheduler include this
 *          module's header. Mcal_TimerTask_Init() still clears the table, so
 *          this has to run after it.
 */
void App_Task_Init(void)
{
    /* The sampling does not run on a slot of this scheduler: it hangs off the
     * hardware base tick, because what it needs is the interval the timer gives
     * it and not the one the main loop reaches. The tick calls it by pointer,
     * so this is the only place the two are connected. */
    Mcal_SampleTick_SetHandler(App_Sensor_Tick);

    Task_Create(Task_10ms_Handle,  Task_10ms_ID,  10U);
    Task_Create(Task_20ms_Handle,  Task_20ms_ID,  20U);
    Task_Create(Task_50ms_Handle,  Task_50ms_ID,  50U);
    Task_Create(Task_200ms_Handle, Task_200ms_ID, 200U);
    Task_Create(Task_300ms_Handle, Task_300ms_ID, 300U);
}
