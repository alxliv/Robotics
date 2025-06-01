#ifndef ALL_TIMER_TASK_H
#define ALL_TIMER_TASK_H

typedef struct TimerTask TimerTask;
typedef void (*timer_task_callback_t)(TimerTask *, uint32_t);

struct TimerTask
{
    uint32_t interval_us;
    uint32_t last_us;
    uint32_t counter;
    timer_task_callback_t callback;
    void *user_data;
};

static void TT_Init(TimerTask *tt, uint32_t interval_us, timer_task_callback_t callback_func)
{
    tt->interval_us = interval_us;
    tt->callback = callback_func;
    tt->last_us = time_us_32();
    tt->counter = 0;
}

static void TT_Update(TimerTask *tt, uint32_t *ptr_now_us)
{
    if (*ptr_now_us - tt->last_us >= tt->interval_us)
    {
        tt->last_us += tt->interval_us;
        if (tt->callback)
        {
            tt->callback(tt, *ptr_now_us);
            tt->counter++;
            *ptr_now_us = time_us_32();
        }
    }
}
#endif
