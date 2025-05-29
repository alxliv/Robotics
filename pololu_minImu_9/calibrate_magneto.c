#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "all_timer_task.h"

const int NUM_SAMPLES = 500;
extern void read_magneto(uint8_t *buf, int nbytes);

typedef struct
{
    uint32_t point_num;
    uint32_t timestamp_us;
    int16_t mX, mY, mZ;
} MAGNETO_DATA;

static void collect_callback(TimerTask *tt, uint32_t now_us)
{
    if (tt->counter >= NUM_SAMPLES)
        return;
    MAGNETO_DATA *dataBuf = (MAGNETO_DATA *)tt->user_data;
    MAGNETO_DATA *pd = &dataBuf[tt->counter];
    pd->point_num = tt->counter;
    pd->timestamp_us = now_us;
    uint8_t raw[6];
    read_magneto(raw, 6);
    int16_t x = (raw[1] << 8) | raw[0];
    int16_t y = (raw[3] << 8) | raw[2];
    int16_t z = (raw[5] << 8) | raw[4];
    pd->mX = x;
    pd->mY = y;
    pd->mZ = z;
}

static void info_callback(TimerTask *tt, uint32_t now_us)
{
    TimerTask *cT = tt->user_data;
    printf("Samples [%d/%d]..\n", cT->counter, NUM_SAMPLES);
}

#define TRIM (0.05f) // trim 5% at each end

// comparison function for qsort
int cmp_int16(const void *a, const void *b)
{
    int16_t ia = *(const int16_t *)a;
    int16_t ib = *(const int16_t *)b;
    return (ia < ib) ? -1 : (ia > ib);
}

// Compute trimmed-min/max on a sorted buffer
void trimmed_min_max(int16_t *buf, int n, float trim,
                     int16_t *min_out, int16_t *max_out)
{
    int i_min = (int)floorf(trim * n);
    int i_max = (int)ceilf((1.0f - trim) * n) - 1;
    if (i_min < 0)
        i_min = 0;
    if (i_max >= n)
        i_max = n - 1;
    *min_out = buf[i_min];
    *max_out = buf[i_max];
}
static void get_min_max(uint16_t *buf, int16_t *pmin, int16_t *pmax)
{
    qsort(buf, NUM_SAMPLES, sizeof(int16_t), cmp_int16);
    trimmed_min_max(buf, NUM_SAMPLES, TRIM, pmin, pmax);
}

static void process_samples(const MAGNETO_DATA *pd)
{
    printf("Processing..\n");
    uint16_t *d = malloc(NUM_SAMPLES * sizeof(uint16_t));
    if (!d)
    {
        printf("malloc (process_samples) failed!\n");
        for (;;)
        {
        }
    }
    int16_t minV, maxV;

    printf("axis X\n");
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        d[i] = pd[i].mX;
    }
    get_min_max(d, &minV, &maxV);
    printf("(%d : %d) Offset=%d\n\n", minV, maxV, (minV + maxV) / 2);
    printf("axis Y\n");
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        d[i] = pd[i].mY;
    }
    get_min_max(d, &minV, &maxV);
    printf("(%d : %d) Offset=%d\n\n", minV, maxV, (minV + maxV) / 2);
    printf("axis Z\n");
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        d[i] = pd[i].mZ;
    }
    get_min_max(d, &minV, &maxV);
    printf("(%d : %d) Offset=%d\n\n", minV, maxV, (minV + maxV) / 2);
    free(d);
    printf("ALL DONE OK.\n");
}

void calibrate_magneto()
{
    char *data = (char *)malloc(sizeof(MAGNETO_DATA));
    if (!data)
    {
        printf("calibrate_magneto() malloc failed!\n");
        return;
    }
    printf("Starting MAGENTOMETER Calibration\n");
    printf("Collecting %d samples\n", NUM_SAMPLES);
    printf("Rotate magentometer around X,Y and Z axis\n");
    TimerTask collectTask;
    TT_Init(&collectTask, 100 * 1000, collect_callback); // 10Hz
    collectTask.user_data = data;
    TimerTask infoTask;
    TT_Init(&infoTask, 1000 * 1000, info_callback); // once a sec
    infoTask.user_data = &collectTask;

    int nloop = 0;
    for (;;)
    {
        nloop++;
        uint32_t now_us = time_us_32();
        TT_Update(&collectTask, now_us);
        if (collectTask.counter >= NUM_SAMPLES)
        {
            break;
        }
        TT_Update(&infoTask, now_us);
    }

    process_samples((MAGNETO_DATA *)data);
    free(data);
}
