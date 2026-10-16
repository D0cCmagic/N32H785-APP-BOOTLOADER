/**
 * @file    Mcal_TimerTask.c
 * @brief   Timebase-driven cooperative software task scheduler.
 * @author  D0cC
 * @date    2026-09-30
 * @version V1.3.0
 *
 * @note    The 1ms interrupt only increments TimeBase.
 *          Task_Handle_Cycle() is called from the main while(1) loop.
 *          All task periods are registered in Mcal_TimerTask_Init().
 *          BTIM1 adds a faster tick on hardware for the sensor sampling, which
 *          a slot of this cooperative scheduler cannot provide.
 */

#include "Mcal_TimerTask.h"
#include "main.h"                   /* provides extern volatile uint32_t TimeBase */
#include "log.h"
#include "log_cfg.h"
#include "n32h7xx_rcc.h"
#include "n32h7xx_tim.h"

static Task_Timer_t task_list[Task_Timer_MaxNum];

/* Owned by the BTIM1 interrupt, read by the console, so it is volatile */
volatile uint32_t Mcal_SampleTickCount = 0u;

/* Divides the base tick down to the sample rate. Only the interrupt touches it,
 * so it needs no protection and no volatile. */
static uint32_t Mcal_SampleDivCount = 0u;

/* How long the tick is measured for before its prescaler is corrected. Long
 * enough that a millisecond of timebase jitter is lost in the rounding, short
 * enough that the init path does not notice it. */
#define MCAL_TICK_CAL_MS                (200u)

/* Cleared until the tick has been measured and corrected, so the filter's first
 * samples are taken at the rate the console reports rather than at whatever the
 * uncorrected prescaler produced. */
static uint8_t Mcal_SampleTickRun = 0u;

/* What to call on a sample tick, registered by the application. Called through a
 * pointer so that this file, which is hardware abstraction, does not have to know
 * what the application does with a sample -- the same arrangement as the task
 * table above. */
static SampleTick_HandleFun Mcal_SampleTickFun = 0;

/**
 * @name    Mcal_SampleTick_SetHandler
 * @brief   Register the function the sample tick calls, once per MCAL_SAMPLE_DIV.
 * @param   fun: the handler; nothing is called until this is set.
 * @retval  None
 */
void Mcal_SampleTick_SetHandler(SampleTick_HandleFun fun)
{
    Mcal_SampleTickFun = fun;
}

/**
 * @name    Mcal_TimerTask_Init
 * @brief   Reset the timebase and register every periodic task slot.
 * @param   None
 * @retval  None
 */
void Mcal_TimerTask_Init(void)
{
    uint8_t i;

    TimeBase = 0U;

    for (i = 0U; i < Task_Timer_MaxNum; i++)
    {
        task_list[i].run_flag      = 0U;
        task_list[i].reload        = 0U;
        task_list[i].last_tick     = 0U;
        task_list[i].Task_Time_Fun = 0;
    }

    /* Every task slot is registered by the application, in App_Task_Init().
     * Which work runs on which period is the application's business, and doing
     * it from here is what made this file depend on the application layer. */

    /* The base tick is deliberately not started here. It runs the sampling, and
     * the parts it reads are configured well after this returns; its start is
     * the last thing Mcal_InitSum() does. */
}

/**
 * @name    Mcal_SampleTickProgram
 * @brief   Load a prescaler into BTIM1 and leave it counting on it.
 * @param   Psc: prescaler value, one or more; the register takes it minus one.
 * @retval  None
 * @note    The period is fixed at 1000 counts, so the interrupt rate is the
 *          counter rate divided by 1000 and the prescaler is the only thing
 *          that decides it.
 */
static void Mcal_SampleTickProgram(uint32_t Psc)
{
    TIM_TimeBaseInitType TimInit;

    TIM_InitTimBaseStruct(&TimInit);
    TimInit.Prescaler   = Psc - 1u;
    TimInit.Period      = 1000u - 1u;
    TimInit.CounterMode = TIM_CNT_MODE_UP;
    TimInit.ClkDiv      = TIM_CLK_DIV1;
    TIM_InitTimeBase(BTIM1, &TimInit);

    /* Cleared after programming: writing the period raises the update event,
     * and an ISR taken for that one would be a phantom tick. */
    TIM_ClrIntPendingBit(BTIM1, TIM_INT_UPDATE);
}

/**
 * @name    Mcal_SampleTick_Init
 * @brief   Start the base sample tick on BTIM1, then measure it and correct it.
 * @param   None
 * @retval  None
 * @note    The prescaler is computed from AHB1 and then measured against the
 *          1 ms SysTick timebase, and that measurement is what settled which
 *          clock this divides: taken from M7ClkFreq the tick ran at 1000 Hz,
 *          and the prescaler the correction produced implies a 300 MHz kernel,
 *          which is AHB1 -- the clock the driver's comment names. Earlier
 *          readings of the same function disagreed because
 *          RCC_GetClocksFreqValue() reports a partly configured tree when it is
 *          called early in the init order, which is why this runs at the end of
 *          it and why the rate is checked rather than assumed.
 * @note    The correction only fires when the measurement is off by more than a
 *          tenth, so the normal case keeps the exactly computed prescaler and
 *          the tick stays a whole division of the clock with no measurement
 *          error in it. A tenth is far outside this window's own resolution and
 *          far inside the factor-of-two surprises this has already produced.
 * @note    Nothing is sampled until the measurement is done and the corrected
 *          prescaler is in force.
 * @note    A base timer has no output pin and no capture channel, so nothing
 *          the board wires up can be sharing it and nothing needs configuring
 *          back to a safe state first. It runs from here on, at the end of the
 *          init order: everything it reads, and the filter that consumes what
 *          it reads, is configured before the first interrupt.
 */
void Mcal_SampleTick_Init(void)
{
    RCC_ClocksTypeDef Clocks;
    uint32_t          Kernel;
    uint32_t          Psc;
    uint32_t          Expected;
    uint32_t          Measured;
    uint32_t          Start;

    RCC_ConfigBTIMKerClk(RCC_BTIMKERCLK_AHB1_DIV1);
    RCC_GetClocksFreqValue(&Clocks);
    RCC_EnableAPB1PeriphClk1(RCC_APB1_PERIPHEN_M7_BTIM1, ENABLE);

    /* One interrupt every 1000 counts, so the prescaler has to land the counter
     * on 1000 times the tick rate. AHB1 is the clock this divides -- the
     * measurement above confirmed it at 300 MHz. */
    Kernel = Clocks.AHB1ClkFreq;
    Psc    = Kernel / (MCAL_SAMPLE_TICK_HZ * 1000u);
    if (Psc == 0u)
    {
        Psc = 1u;
    }
    Mcal_SampleTickProgram(Psc);

    TIM_ConfigInt(BTIM1, TIM_INT_UPDATE, ENABLE);
    TIM_Enable(BTIM1, ENABLE);

    /* Below SysTick and the radio's SPI: this tick can afford to be late, while
     * they cannot, and a tick that slips by the odd microsecond is worth more
     * than a network interrupt that was held off. */
    NVIC_SetPriority(BTIM1_IRQn, 5);
    NVIC_EnableIRQ(BTIM1_IRQn);

    /* Counted over a fixed span of the timebase that every other part of this
     * firmware already trusts, so the number is in real milliseconds. */
    Start = TimeBase;
    while ((TimeBase - Start) < MCAL_TICK_CAL_MS)
    {
    }
    Measured = Mcal_SampleTickCount;

    if (Measured != 0u)
    {
        Expected = (MCAL_SAMPLE_TICK_HZ * MCAL_TICK_CAL_MS) / 1000u;

        /* Taken only when the measurement says the assumption was wrong by a
         * factor. An exactly computed prescaler beats a measured one, because
         * this window can only resolve about half a percent, and correcting a
         * rate that was already right would trade an exact division for that
         * half percent. */
        if ((Measured > ((Expected * 11u) / 10u)) ||
            (Measured < ((Expected * 9u) / 10u)))
        {
            Psc = (Psc * Measured) / Expected;
            if (Psc == 0u)
            {
                Psc = 1u;
            }
            Mcal_SampleTickProgram(Psc);
        }
    }

    /* Restarted from here, so that the console's rate line measures the rate in
     * force rather than the one the calibration ran at. */
    Mcal_SampleTickCount = 0u;
    Mcal_SampleDivCount  = 0u;
    Mcal_SampleTickRun   = 1u;

#if (LOG_PERIODIC_ENABLE != 0)
    /* Once, at bring-up -- but it lands while the link may already be live, so
     * it is behind the same switch as the repeating lines. See log_cfg.h. */
    log_info("[TICK] m7=%u ahb1=%u apb1=%u psc=%u, raw %u Hz\r\n",
             (unsigned)Clocks.M7ClkFreq, (unsigned)Clocks.AHB1ClkFreq,
             (unsigned)Clocks.APB1ClkFreq, (unsigned)Psc,
             (unsigned)((Measured * 1000u) / MCAL_TICK_CAL_MS));
#else
    (void)Clocks;
    (void)Psc;
    (void)Measured;
#endif
}

/**
 * @name    BTIM1_IRQHandler
 * @brief   One base tick: every MCAL_SAMPLE_DIV of them, sample and fuse.
 * @param   None
 * @retval  None
 * @note    Takes the startup file's weak BTIM1_IRQHandler slot, so the vector
 *          table is left alone -- unlike SysTick, which this image has to
 *          redirect because the vendor links its handler at a slot of its own.
 * @note    The work is done here rather than flagged to the main loop because
 *          that loop is shared with the radio, the network and the video code:
 *          a slot in it measures their schedule, and the filter needs the
 *          interval it actually got. Dividing a hardware tick by a counter
 *          keeps the sample rate an exact ratio of the clock.
 * @note    Long by interrupt standards -- the I2C transfer inside runs to a few
 *          hundred microseconds -- which is why the priority sits below SysTick
 *          and the radio's SPI. Nothing in the call chain prints, because the
 *          console waits on the transmitter character by character.
 */
void BTIM1_IRQHandler(void)
{
    TIM_ClrIntPendingBit(BTIM1, TIM_INT_UPDATE);

    Mcal_SampleTickCount++;

    Mcal_SampleDivCount++;
    Mcal_SampleDivCount %= MCAL_SAMPLE_DIV;
    if ((Mcal_SampleDivCount == 0u) && (Mcal_SampleTickRun != 0u) &&
        (Mcal_SampleTickFun != 0))
    {
        Mcal_SampleTickFun();
    }
}

/**
 * @name    Task_Create
 * @brief   Bind a handler to a slot and start it with the given period.
 * @param   fun: handler to call when the period expires.
 * @param   Task_ID: slot index, below Task_Timer_MaxNum.
 * @param   Task_Time: period in milliseconds, must be non-zero.
 * @retval  None
 */
void Task_Create(Task_Time_HandleFun fun, Task_Handle_ID Task_ID, uint32_t Task_Time)
{
    if ((Task_ID >= Task_Timer_MaxNum) || (fun == 0) || (Task_Time == 0U))
    {
        return;
    }

    task_list[Task_ID].Task_Time_Fun = fun;
    task_list[Task_ID].reload        = Task_Time;
    task_list[Task_ID].last_tick     = TimeBase;
    task_list[Task_ID].run_flag      = 1U;
}

/**
 * @name    Task_Stop
 * @brief   Suspend a slot without losing its registration.
 * @param   Task_ID: slot index, below Task_Timer_MaxNum.
 * @retval  None
 */
void Task_Stop(Task_Handle_ID Task_ID)
{
    if (Task_ID >= Task_Timer_MaxNum)
    {
        return;
    }
    task_list[Task_ID].run_flag = 0U;
}

/**
 * @name    Task_Run
 * @brief   Resume a suspended slot and restart its period from now.
 * @param   Task_ID: slot index, below Task_Timer_MaxNum.
 * @retval  None
 */
void Task_Run(Task_Handle_ID Task_ID)
{
    if (Task_ID >= Task_Timer_MaxNum)
    {
        return;
    }
    task_list[Task_ID].run_flag  = 1U;
    task_list[Task_ID].last_tick = TimeBase;
}

/**
 * @name    Task_Handle_Cycle
 * @brief   Run every slot whose period has expired; call from the main loop.
 * @param   None
 * @retval  None
 */
void Task_Handle_Cycle(void)
{
    uint8_t  i;
    uint32_t now;

    now = TimeBase;

    for (i = 0U; i < Task_Timer_MaxNum; i++)
    {
        if (task_list[i].run_flag == 0U)
        {
            continue;
        }
        if (task_list[i].Task_Time_Fun == 0)
        {
            continue;
        }

        if ((now - task_list[i].last_tick) >= task_list[i].reload)
        {
            task_list[i].last_tick += task_list[i].reload;

            /* Resynchronize if the slot fell more than one period behind */
            if ((now - task_list[i].last_tick) >= task_list[i].reload)
            {
                task_list[i].last_tick = now;
            }

            task_list[i].Task_Time_Fun();
        }
    }
}
