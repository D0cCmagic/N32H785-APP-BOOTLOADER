#ifndef _MCAL_TIMERTASK_H_
#define _MCAL_TIMERTASK_H_

#include <stdint.h>

#define Task_Timer_MaxNum        5

typedef void (*Task_Time_HandleFun)(void);

typedef void (*SampleTick_HandleFun)(void);

typedef struct
{
    uint8_t             run_flag;
    uint32_t            reload;
    uint32_t            last_tick;
    Task_Time_HandleFun Task_Time_Fun;
} Task_Timer_t;

typedef enum
{
    Task_10ms_ID,
    Task_20ms_ID,
    Task_50ms_ID,
    Task_200ms_ID,
    Task_300ms_ID,
} Task_Handle_ID;

/* 1. The base sample tick.
 *
 *    A hardware tick for the sensor sampling to hang off, so that its period
 *    is the timer's and not the main loop's. The loop the sensor poll runs in
 *    also carries the radio, the network and the video work, so what a
 *    software slot measures is their schedule rather than its own.
 *
 *    BTIM1 is a base timer: it drives no pin and owns no channel, so it cannot
 *    collide with anything the board wires up. Its prescaler is worked out from
 *    the clock the RCC reports and then corrected against the 1 ms timebase,
 *    because which clock reaches a base timer on this part turned out to depend
 *    on where in the init order the configuration ran -- see
 *    Mcal_SampleTick_Init().
 *
 *    The sampling runs in the interrupt and is divided down from the base rate
 *    by a counter, so the sample rate is an exact ratio of the clock too rather
 *    than a slot the main loop has to reach. The [TICK] line printed once at
 *    init records the clocks the prescaler was computed from, the prescaler it
 *    ended on, and the rate it measured before any correction -- which is where
 *    that check against the timebase is kept. */
#define MCAL_SAMPLE_TICK_HZ         (2000u)
#define MCAL_SAMPLE_DIV             (16u)       /* 2000 / 16 = 125 Hz          */

extern volatile uint32_t Mcal_SampleTickCount;

void Mcal_SampleTick_Init(void);
void Mcal_SampleTick_SetHandler(SampleTick_HandleFun fun);

void Mcal_TimerTask_Init(void);
void Task_Create(Task_Time_HandleFun fun, Task_Handle_ID Task_ID, uint32_t Task_Time);
void Task_Stop(Task_Handle_ID Task_ID);
void Task_Run(Task_Handle_ID Task_ID);
void Task_Handle_Cycle(void);

#endif /* _MCAL_TIMERTASK_H_ */
