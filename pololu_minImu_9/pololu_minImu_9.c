#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "all_timer_task.h"

// #define CALIBRATE_MAGNETOMETER // comment that out for a normal operation

#define GYRO_ACCEL_ADDR (0x6B)
#define MAGNETO_METER_ADDR (0x1E) // LIS3MDL sensor

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define ODR_10HZ 0x10 // DO[2:0]=100 → 10 Hz, TEMP_EN=0, FAST_ODR=0
#define CTRL_REG3 0x22
#define CONTINUOUS_MODE 0x00 // MD[1:0]=00

/****************************
 * CTRL_REG1 bits:
| Bit(s) | Name      | Description                                 | Values                                                                                                                                              |
| :----: | :-------- | :------------------------------------------ | :-------------------------------------------------------------------------------------------------------------------------------------------------- |
|    7   | TEMP\_EN  | Temperature sensor enable                   | 0 = disabled (default)<br>1 = enabled ([STMicroelectronics][1])                                                                                     |
|   6–5  | OM$1:0$   | X/Y-axes operating mode selection           | 00 = Low-power<br>01 = Medium-performance<br>10 = High-performance<br>11 = Ultra-high-performance ([STMicroelectronics][1])                         |
|   4–2  | DO$2:0$   | Output data-rate selection                  | 000 = 0.625 Hz<br>001 = 1.25 Hz<br>010 = 2.5 Hz<br>011 = 5 Hz<br>100 = 10 Hz<br>101 = 20 Hz<br>110 = 40 Hz<br>111 = 80 Hz ([STMicroelectronics][1]) |
|    1   | FAST\_ODR | Fast-output-data-rate enable (beyond 80 Hz) | 0 = disabled (≤ 80 Hz)<br>1 = enabled (1000/560/300/155 Hz depending on OM) ([STMicroelectronics][1])                                               |
|    0   | ST        | Self-test enable                            | 0 = disabled (default)<br>1 = enabled ([STMicroelectronics][1])                                                                                     |

[1]: https://www.st.com/resource/en/datasheet/lis3mdl.pdf "Datasheet - LIS3MDL - Digital output magnetic sensor:  ultralow-power, high-performance 3-axis magnetometer"
*****************************/
extern void calibrate_magneto();

static void write_mag_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    i2c_write_blocking(i2c_default, MAGNETO_METER_ADDR, buf, 2, false);
}

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
    int16_t min_mX, min_mY, min_mZ;
    int16_t max_mX, max_mY, max_mZ;
} MAGNETO_DATA;

MAGNETO_DATA magnetoData = {0, 0, 0, 0, 0,
            32767, 32767, 32767,
            -32768, -32768, -32768
};

#define min(a,b) ((a)<(b))?(a):(b);
#define max(a,b) ((a)>(b))?(a):(b);

void read_magneto(uint8_t *buf, int nbytes)
{
    // Burst-read six data bytes (X_L…), convert, print
    uint8_t reg = 0x28 | 0x80; // OUT_X_L with auto-increment
    i2c_write_blocking(i2c_default, MAGNETO_METER_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MAGNETO_METER_ADDR, buf, nbytes, false);
}

void magneto_callback(TimerTask *tt, uint32_t now_us)
{
    magnetoData.point_num = tt->counter;
    magnetoData.timestamp_us = now_us;

    // Burst-read six data bytes (X_L…), convert, print
    uint8_t reg = 0x28 | 0x80; // OUT_X_L with auto-increment
    uint8_t raw[6];
    read_magneto(raw, 6);
    int16_t x = (raw[1] << 8) | raw[0];
    int16_t y = (raw[3] << 8) | raw[2];
    int16_t z = (raw[5] << 8) | raw[4];
    magnetoData.mX = x;
    magnetoData.min_mX = min(magnetoData.min_mX, x);
    magnetoData.max_mX = max(magnetoData.max_mX, x);

    magnetoData.mY = y;
    magnetoData.min_mY = min(magnetoData.min_mY, y);
    magnetoData.max_mY = max(magnetoData.max_mY, y);

    magnetoData.mZ = z;
    magnetoData.min_mZ = min(magnetoData.min_mZ, z);
    magnetoData.max_mZ = max(magnetoData.max_mZ, z);
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
                    devname = "GYRO_ACCEL_ADDR";
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

void info_callback(TimerTask *tt, uint32_t now_us)
{
    printf("[%d]: x=%d(%d:%d), y=%d(%d:%d), z=%d(%d:%d)\n", magnetoData.point_num,
        magnetoData.mX, magnetoData.min_mX, magnetoData.max_mX,
        magnetoData.mY, magnetoData.min_mY, magnetoData.max_mY,
        magnetoData.mZ, magnetoData.min_mZ, magnetoData.max_mZ
    );
}

static void led_callback(TimerTask *tt, uint32_t now_us)
{
    bool onoff = (tt->counter % 2 == 0);
    pico_set_led(onoff);
}

static void set_mag_default()
{
    // --- Configure LIS3MDL for 10 Hz continuous mode ---
    write_mag_reg(CTRL_REG1, 0x70); // ultra-high-performance for X and Y + 10Hz ODR
    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    write_mag_reg(CTRL_REG2, 0x00);
    write_mag_reg(CTRL_REG3, CONTINUOUS_MODE);
    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    write_mag_reg(CTRL_REG4, 0x0C);

    // 0x40 = 0b01000000
    // BDU = 1 (block data update)
    write_mag_reg(CTRL_REG5, 0x40);
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

    set_mag_default();

    sleep_ms(3000);
#ifdef CALIBRATE_MAGNETOMETER
    calibrate_magneto();
    for (;;);
#endif

    TimerTask magnetoTask;
    TT_Init(&magnetoTask, 100 * 1000, magneto_callback); // 10Hz
    TimerTask infoTask;
    TT_Init(&infoTask, 1000 * 1000, info_callback);
    TimerTask ledTask;
    TT_Init(&ledTask, 500 * 1000, led_callback);

    printf("v0.1 minIMU-9 started.\n");
    int nloop = 0;
    while (true)
    {
        nloop++;
        uint32_t now_us = time_us_32();
        TT_Update(&magnetoTask, now_us);
        TT_Update(&infoTask, now_us);
        TT_Update(&ledTask, now_us);
    }
}
