#include <stdio.h>
#include "pico/stdlib.h"
#include "mag_lis3mdl.h"

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

const int OFFS_X = 0;
const int OFFS_Y = 1000;
const int OFFS_Z = 0;

/*
 * Stub: write 'len' bytes from 'data' to device 'dev_addr' starting at register 'reg_addr'.
 * Return 0 on success, non-zero on failure. Replace with your platform's I2C write routine.
 */
extern int i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);

/*
 * Stub: read 'len' bytes into 'data' from device 'dev_addr' starting at register 'reg_addr'.
 * Return 0 on success, non-zero on failure. Replace with your platform's I2C read routine.
 */
extern int i2c_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

/*----------------------------------------------------------------------
 * 3) Helper: write a single register
 *----------------------------------------------------------------------*/
static int lis3msl_write_reg(uint8_t reg_addr, uint8_t value)
{
    return i2c_write_reg(LIS3MDL_I2C_ADDR, reg_addr, value);
}

/*----------------------------------------------------------------------
 * 4) Helper: read a single register
 *----------------------------------------------------------------------*/
static int lis3mdl_read_reg(uint8_t reg_addr, uint8_t *value)
{
    return i2c_read_regs(LIS3MDL_I2C_ADDR, reg_addr, value, 1);
}

/*----------------------------------------------------------------------
 * 5) Combine two bytes into a signed 16-bit integer (little-endian)
 *----------------------------------------------------------------------*/
static int16_t combine_bytes(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

int lis3mdl_init()
{

    // --- Configure LIS3MDL for 10 Hz continuous mode ---
    lis3msl_write_reg(CTRL_REG1, 0x70); // ultra-high-performance for X and Y + 10Hz ODR
    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    lis3msl_write_reg(CTRL_REG2, 0x00);
    lis3msl_write_reg(CTRL_REG3, CONTINUOUS_MODE);
    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    lis3msl_write_reg(CTRL_REG4, 0x0C);

    // 0x40 = 0b01000000
    // BDU = 1 (block data update)
    lis3msl_write_reg(CTRL_REG5, 0x40);

    return 0;
}

int lis3mdl_read(int16_t *out_x, int16_t *out_y, int16_t *out_z)
{
    int ret;
    uint8_t buf[6];

    // Burst-read six data bytes (X_L…), convert, print
    uint8_t reg = 0x28 | 0x80; // OUT_X_L with auto-increment

    /* Read 6 bytes starting from OUTX_L_XL (auto-increment is enabled) */
    ret = i2c_read_regs(LIS3MDL_I2C_ADDR, reg, buf, 6);
    if (ret != 0)
    {
        return ret;
    }

    *out_x = combine_bytes(buf[0], buf[1]) + OFFS_X;
    *out_y = combine_bytes(buf[2], buf[3]) + OFFS_Y;
    *out_z = combine_bytes(buf[4], buf[5]) + OFFS_Z;

    return 0;
}
