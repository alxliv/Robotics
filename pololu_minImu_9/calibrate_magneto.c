#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "all_timer_task.h"
#include "gyro_accel.h"
#include "mag_lis3mdl.h"
#include "calibrate_magneto.h"

const int NUM_SAMPLES = 500;

static int16_t minX, maxX, minY, maxY, minZ, maxZ;
static float biasX, biasY, biasZ;
static float scaleX, scaleY, scaleZ;
static float radiusFactory = 0.0f; // optional: if you know Earth’s field magnitude here

typedef struct
{
    uint32_t point_num;
    uint32_t timestamp_us;
    int16_t mX, mY, mZ;
} MAGNETO_DATA;

void load_magneto_calibration(uint16_t ver)
{

    if (ver==LSM6DSO_WHO_AM_I_ID)
    {
        biasX=596.50;
        scaleX=0.905;
        biasY=-527.00;
        scaleY=1.046;
        biasZ=-381.50;
        scaleZ=1.064;
    } else {
        biasX=2843.00; scaleX=1.029;
        biasY=-4795.50; scaleY=1.015;
        biasZ=9883.50; scaleZ=0.959;
    }

}

static void collect_callback(TimerTask *tt, uint32_t now_us)
{
    if (tt->counter >= NUM_SAMPLES)
        return;
    MAGNETO_DATA *dataBuf = (MAGNETO_DATA *)tt->user_data;
    MAGNETO_DATA *pd = &dataBuf[tt->counter];
    pd->point_num = tt->counter;
    pd->timestamp_us = now_us;
    if (lis3mdl_read(&pd->mX, &pd->mY, &pd->mZ) != 0)
    {
        printf("is3mdl_read() failed. STOP.\n");
        hard_assert(false);
    }
}
static uint16_t data_samples = 0;

static void data_callback(TimerTask *tt, uint32_t now_us)
{
    int16_t mx, my, mz;

    if (tt->counter >= NUM_SAMPLES)
        return;
    data_samples = tt->counter;
    if (lis3mdl_read(&mx, &my, &mz) != 0)
    {
        printf("is3mdl_read() failed. STOP.\n");
        hard_assert(false);
    }
    if (mx < minX)
        minX = mx;
    if (mx > maxX)
        maxX = mx;
    if (my < minY)
        minY = my;
    if (my > maxY)
        maxY = my;
    if (mz < minZ)
        minZ = mz;
    if (mz > maxZ)
        maxZ = mz;
}

static void info_callback(TimerTask *tt, uint32_t now_us)
{
    TimerTask *cT = tt->user_data;
    printf("Samples [%d/%d]..\n", data_samples, NUM_SAMPLES);
}

static void init(void)
{
    minX = INT16_MAX;
    maxX = -INT16_MAX;
    minY = INT16_MAX;
    maxY = -INT16_MAX;
    minZ = INT16_MAX;
    maxZ = -INT16_MAX;
}

static void compute_calibration()
{
    // 3.1 Hard-iron biases
    biasX = (maxX + minX) * 0.5f;
    biasY = (maxY + minY) * 0.5f;
    biasZ = (maxZ + minZ) * 0.5f;

    // 3.2 Half-ranges
    float Rx = (maxX - minX) * 0.5f;
    float Ry = (maxY - minY) * 0.5f;
    float Rz = (maxZ - minZ) * 0.5f;
    float R_avg = (Rx + Ry + Rz) / 3.0f;

    // 3.4 Soft-iron scale factors
    scaleX = R_avg / Rx;
    scaleY = R_avg / Ry;
    scaleZ = R_avg / Rz;

    // Print results for debugging
    printf("*** Magnetometer calibration results:\n");

    printf("minX=%d, maxX=%d\n", minX, maxX);
    printf("minY=%d, maxY=%d\n", minY, maxY);
    printf("minZ=%d, maxZ=%d\n", minZ, maxZ);

    printf("\n== Insert into calibrate_magneto.c load_magneto_calibration(): ==\n");
    printf("biasX=%.2f; scaleX=%.3f;\n", biasX, scaleX);
    printf("biasY=%.2f; scaleY=%.3f;\n", biasY, scaleY);
    printf("biasZ=%.2f; scaleZ=%.3f;\n", biasZ, scaleZ);
    printf("==========================================================\n");
}

void calibrate_magneto()
{
    printf("Starting MAGENTOMETER Calibration\n");
    printf("Will run for %d samples\n", NUM_SAMPLES);
    printf("Rotate magentometer around X,Y and Z axis\n");
    init();

    TimerTask dataTask;
    TT_Init(&dataTask, 100 * 1000, data_callback); // 10Hz
    TimerTask infoTask;
    TT_Init(&infoTask, 1000 * 1000, info_callback); // once a sec

    for (;;)
    {
        uint32_t now_us = time_us_32();
        TT_Update(&dataTask, &now_us);
        if (dataTask.counter >= NUM_SAMPLES)
        {
            break;
        }
        TT_Update(&infoTask, &now_us);
    }

    //    process_samples((MAGNETO_DATA *)data);
    compute_calibration();
}

void magnetometer_apply_calibration(int16_t mx_raw, int16_t my_raw, int16_t mz_raw,
                                    float *mx_cal, float *my_cal, float *mz_cal)
{
    // 4.1 Subtract hard-iron bias (in raw counts)
    float mx_centered = (float)mx_raw - biasX;
    float my_centered = (float)my_raw - biasY;
    float mz_centered = (float)mz_raw - biasZ;

    // 4.2 Apply scale factors to remove soft-iron
    *mx_cal = mx_centered * scaleX;
    *my_cal = my_centered * scaleY;
    *mz_cal = mz_centered * scaleZ;
}
