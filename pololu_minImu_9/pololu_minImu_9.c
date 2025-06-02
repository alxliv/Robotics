#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "all_timer_task.h"
#include "gyro_accel.h"
#include "mag_lis3mdl.h"
#include "calibrate_magneto.h"

// #define CALIBRATE_MAGNETOMETER // comment that out for a normal operation

#define GYRO_ACCEL_ADDR LSM6DSO_I2C_ADDR
#define MAGNETO_METER_ADDR LIS3MDL_I2C_ADDR // magnetometer LIS3MDL sensor

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

typedef struct
{
    uint32_t point_num;
    uint32_t timestamp_us;
    int16_t mX, mY, mZ;
} SENSOR_DATA;

SENSOR_DATA magnetoData = {0, 0, 0, 0, 0};
SENSOR_DATA gyroData = {0, 0, 0, 0, 0};
SENSOR_DATA accelData = {0, 0, 0, 0, 0};

int i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg_addr;
    buf[1] = value;
    int retc = i2c_write_blocking(i2c_default, dev_addr, buf, 2, false);
    if (retc != 2)
    {
        printf("i2c_write failed (len=%d, retc=%d)\n", 2, retc);
        hard_assert(false);
    }
    return 0;
}

int i2c_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int retc = i2c_write_blocking(i2c_default, dev_addr, &reg_addr, 1, true);
    if (retc != 1)
    {
        printf("i2c_write failed (len=%d, retc=%d)\n", 1, retc);
//        hard_assert(false);
    }
    retc = i2c_read_blocking(i2c_default, dev_addr, data, len, false);
    if (retc != len)
    {
        printf("i2c_read failed (len=%d, retc=%d)\n", len, retc);
 //       hard_assert(false);
    }
    return 0;
}

void magneto_callback(TimerTask *tt, uint32_t now_us)
{
    SENSOR_DATA *sd = &magnetoData;
    sd->point_num = tt->counter;
    sd->timestamp_us = now_us;
    if (lis3mdl_read(&sd->mX, &sd->mY, &sd->mZ) != 0)
    {
        printf("is3mdl_read() failed. STOP.\n");
        hard_assert(false);
    }
}

void gyro_callback(TimerTask *tt, uint32_t now_us)
{
    SENSOR_DATA *sd = &gyroData;
    sd->point_num = tt->counter;
    sd->timestamp_us = now_us;
    lsm6dso_read_gyro_mdps(&sd->mX, &sd->mY, &sd->mZ);
}

void accel_callback(TimerTask *tt, uint32_t now_us)
{
    SENSOR_DATA *sd = &accelData;
    sd->point_num = tt->counter;
    sd->timestamp_us = now_us;
    lsm6dso_read_accel_mg(&sd->mX, &sd->mY, &sd->mZ);
}

// Perform initialisation
int pico_led_init(void)
{
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on)
{
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr)
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

bool magneto_ok = false;
bool gyro_accel_ok = false;

void i2c_scan(i2c_inst_t *i2c)
{
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 0; addr <= 127; addr++)
    {
        uint8_t dummy = 0;
        if (!reserved_addr(addr))
        {
            int ret = i2c_read_blocking(i2c, addr, &dummy, 1, false);
            if (ret >= 0)
            {
                char *devname = "UNKNOWN";
                if (addr == GYRO_ACCEL_ADDR)
                {
                    devname = "GYRO_ACCEL_ADDR";
                    gyro_accel_ok = true;
                }
                else if (addr == MAGNETO_METER_ADDR)
                {
                    devname = "MAGNETO_METER_ADDR";
                    magneto_ok = true;
                }
                printf("I2C Device (%s) found at 0x%02X\n", devname, addr);
            }
        }
    }
    printf("Scan complete.\n");
}

/*
 * Given:
 *   ax, ay, az = raw accelerometer readings (e.g. in [g] or any units proportional to gravity).
 *   mx, my, mz = raw magnetometer readings (e.g. in μT or any units proportional to the Earth field).
 *
 * Outputs:
 *   out_nx, out_ny, out_nz = a unit-length vector pointing toward magnetic north, in the device’s body frame.
 *
 * Returns 0 on success, -1 if any vector is zero length or invalid.
 */

int compute_tilt_compensated_north(
    float ax, float ay, float az,
    float mx, float my, float mz,
    float *out_nx, float *out_ny, float *out_nz,
    float *heading_deg)
{
    // 1) Normalize accelerometer: g = accel / ||accel||
    float normA = sqrtf(ax * ax + ay * ay + az * az);
    if (normA < 1e-6f)
        return -1; // invalid accel vector
    float gx = ax / normA;
    float gy = ay / normA;
    float gz = az / normA;

    // 2) Normalize magnetometer: m = mag / ||mag||
    float normM = sqrtf(mx * mx + my * my + mz * mz);
    if (normM < 1e-6f)
        return -1; // invalid mag vector
    float mxn = mx / normM;
    float myn = my / normM;
    float mzn = mz / normM;

    // 3) East = m × g
    float Ex = myn * gz - mzn * gy;
    float Ey = mzn * gx - mxn * gz;
    float Ez = mxn * gy - myn * gx;
    float normE = sqrtf(Ex * Ex + Ey * Ey + Ez * Ez);
    if (normE < 1e-6f)
        return -1; // device is exactly aligned with magnetic field?

    Ex /= normE;
    Ey /= normE;
    Ez /= normE;

    // 4) North = g × E
    float Nx = gy * Ez - gz * Ey;
    float Ny = gz * Ex - gx * Ez;
    float Nz = gx * Ey - gy * Ex;
    float normN = sqrtf(Nx * Nx + Ny * Ny + Nz * Nz);
    if (normN < 1e-6f)
        return -1;

    Nx /= normN;
    Ny /= normN;
    Nz /= normN;

    // 5) Output
    *out_nx = Nx;
    *out_ny = Ny;
    *out_nz = Nz;
    // Suppose the device’s forward-facing axis is +X in body frame,
    // and we want heading = 0° when pointing toward magnetic north.
    float heading_rad = atan2f(Ex, Nx);
    // Convert to [0, 2π):
    if (heading_rad < 0)
        heading_rad += 2.0f * (float)M_PI;
    *heading_deg = heading_rad * 180.0f / (float)M_PI;
    return 0;
}

void info_callback(TimerTask *tt, uint32_t now_us)
{
#if 0
//    SENSOR_DATA *sd = &accelData;
    // SENSOR_DATA *sd = &gyroData;
    SENSOR_DATA *sd = &magnetoData;

    printf("%d,%d,%d\n", sd->mX, sd->mY, sd->mZ);

#endif

#if 1
    float out_nx, out_ny, out_nz;
    float heading_deg;

    float mx, my, mz;
    magnetometer_apply_calibration(magnetoData.mX, magnetoData.mY, magnetoData.mZ, &mx, &my, &mz);

    if (compute_tilt_compensated_north(
            accelData.mX, accelData.mY, accelData.mZ,
            mx, my, mz,
            &out_nx, &out_ny, &out_nz,
            &heading_deg) != 0)
    {
        printf("compute_tilt_failed!\n");
        return;
    }

    printf("%f\n", heading_deg);
#endif
}

static void led_callback(TimerTask *tt, uint32_t now_us)
{
    bool onoff = (tt->counter % 2 == 0);
    pico_set_led(onoff);
}

int main()
{

    stdio_init_all();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // Initialize I2C0 at 400 kHz on the default pins (GP4 = SDA, GP5 = SCL)
    i2c_init(i2c_default, 400 * 1000);

    // Set up I2C pins (example for Pico)
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C); // SDA GP4 which is physical pin 6
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C); // SCL GP5 - physical pin 7
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    i2c_scan(i2c_default);
    if (!magneto_ok)
    {
        printf("ERROR: Magnetometer device NOT found!. STOP!!!\n");
        hard_assert(false);
    }
    if (!gyro_accel_ok)
    {
        printf("ERROR: GYRO_ACCEL device NOT found!. STOP!!!\n");
        hard_assert(false);
    }

    lis3mdl_init();
    lsm6dso_init();

    sleep_ms(3000);
    load_magneto_calibration();

 #ifdef CALIBRATE_MAGNETOMETER
    calibrate_magneto();
#endif

    TimerTask magnetoTask;
    TT_Init(&magnetoTask, 100 * 1000, magneto_callback); // 10Hz
    TimerTask gyroTask;
    TT_Init(&gyroTask, 100 * 1000, gyro_callback); // 10Hz
    TimerTask accelTask;
    TT_Init(&accelTask, 100 * 1000, accel_callback); // 10Hz

    TimerTask infoTask;
    TT_Init(&infoTask, 250 * 1000, info_callback);
    TimerTask ledTask;
    TT_Init(&ledTask, 500 * 1000, led_callback);

    printf("v0.1 minIMU-9 started.\n");
    while (true)
    {
        uint32_t now_us = time_us_32();
        TT_Update(&magnetoTask, &now_us);
        TT_Update(&gyroTask, &now_us);
        TT_Update(&accelTask, &now_us);
        TT_Update(&infoTask, &now_us);
        TT_Update(&ledTask, &now_us);
    }
}
