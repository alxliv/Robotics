#include <stdio.h>
#include "pico/stdlib.h"
#include "gyro_accel.h"

/* WHO_AM_I register should return 0x6C for LSM6DSO */
#define LSM6DSO_WHO_AM_I      0x0F
#define LSM6DSO_WHO_AM_I_ID   0x6C

/* Control registers */
#define LSM6DSO_CTRL1_XL      0x10  /* Accel: ODR, FS, LPF1 */
#define LSM6DSO_CTRL2_G       0x11  /* Gyro: ODR, FS */
#define LSM6DSO_CTRL3_C       0x12  /* Misc settings: IF_INC auto-increment, BDU, etc. */

/* Output registers (auto-increment if IF_INC = 1) */
#define LSM6DSO_OUTX_L_G      0x22  /* Gyro X LSB */
#define LSM6DSO_OUTX_H_G      0x23  /* Gyro X MSB */
#define LSM6DSO_OUTY_L_G      0x24
#define LSM6DSO_OUTY_H_G      0x25
#define LSM6DSO_OUTZ_L_G      0x26
#define LSM6DSO_OUTZ_H_G      0x27

#define LSM6DSO_OUTX_L_XL     0x28  /* Accel X LSB */
#define LSM6DSO_OUTX_H_XL     0x29  /* Accel X MSB */
#define LSM6DSO_OUTY_L_XL     0x2A
#define LSM6DSO_OUTY_H_XL     0x2B
#define LSM6DSO_OUTZ_L_XL     0x2C
#define LSM6DSO_OUTZ_H_XL     0x2D

/* Sensitivity (from datasheet) for FS = ±2 g and FS = ±250 dps */
#define LSM6DSO_SENSITIVITY_ACCEL_2G    (0.061f)   /* mg/LSB (0.061 mg/LSB) */
#define LSM6DSO_SENSITIVITY_GYRO_250DPS (8.75f)    /* mdps/LSB (8.75 mdps/LSB) */

/*----------------------------------------------------------------------
 * 2) Low-level I²C read/write stubs (platform‐specific!)
 *----------------------------------------------------------------------*/

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
static int lsm6dso_write_reg(uint8_t reg_addr, uint8_t value) {
    return i2c_write_reg(LSM6DSO_I2C_ADDR, reg_addr, value);
}

/*----------------------------------------------------------------------
 * 4) Helper: read a single register
 *----------------------------------------------------------------------*/
static int lsm6dso_read_reg(uint8_t reg_addr, uint8_t *value) {
    return i2c_read_regs(LSM6DSO_I2C_ADDR, reg_addr, value, 1);
}

/*----------------------------------------------------------------------
 * 5) Combine two bytes into a signed 16-bit integer (little-endian)
 *----------------------------------------------------------------------*/
static int16_t combine_bytes(uint8_t low, uint8_t high) {
    return (int16_t)((uint16_t)high << 8 | low);
}

/*----------------------------------------------------------------------
 * 6) Initialize LSM6DSO: enable auto-increment, set ODR and full-scale
 *----------------------------------------------------------------------*/
int lsm6dso_init(void) {
    int ret;
    uint8_t who_am_i = 0;

    /* 6.1) Check WHO_AM_I */
    ret = lsm6dso_read_reg(LSM6DSO_WHO_AM_I, &who_am_i);
    if (ret != 0) {
        return ret;  /* I2C error */
    }
    if (who_am_i != LSM6DSO_WHO_AM_I_ID) {
        printf("who_am_i is not LSM6DSO_WHO_AM_I_ID (%d), but %d\n", LSM6DSO_WHO_AM_I_ID, who_am_i);
        return -1;    /* Unexpected device ID */
    }
/*
     * 6.2) Set CTRL3_C:
     *   - BDU = 1    (Block data update: output registers not updated until both high and low bytes read)
     *   - IF_INC = 1 (Register auto-increment enabled so that multi-byte read works across consecutive regs)
     *   - Keep other bits as default (e.g. IF_CS_PU_DIS = 0, etc.)
     */
    uint8_t ctrl3_c = (1 << 6) |  /* BDU = 1 */
                      (1 << 2);   /* IF_INC = 1 */
    ret = lsm6dso_write_reg(LSM6DSO_CTRL3_C, ctrl3_c);
    if (ret != 0) {
        return ret;
    }

    /*
     * 6.3) Configure accelerometer in CTRL1_XL (ODR = 104 Hz, FS = ±2 g)
     *   ODR_XL[3:0]  = 0100  → 104 Hz
     *   FS_XL[1:0]   = 00    → ±2 g
     *   LPF1_BW_SEL  = 0     (use LPF1 at ODR/2 if needed; here keep default)
     */
    uint8_t ctrl1_xl = (0x4 << 4)  /* ODR_XL = 0100 (104 Hz) */
                     | (0x0 << 2)  /* FS_XL = 00 (±2 g) */
                     /* LPF1_BW_SEL = 0 by default */;
    ret = lsm6dso_write_reg(LSM6DSO_CTRL1_XL, ctrl1_xl);
    if (ret != 0) {
        return ret;
    }

    /*
     * 6.4) Configure gyroscope in CTRL2_G (ODR = 104 Hz, FS = 250 dps)
     *   ODR_G[3:0]  = 0100  → 104 Hz
     *   FS_G[1:0]   = 00    → 250 dps
     */
    uint8_t ctrl2_g = (0x4 << 4)  /* ODR_G = 0100 (104 Hz) */
                    | (0x0 << 2)  /* FS_G = 00 (250 dps) */
                    /* HP_SLOPE_XL_EN, etc., default = 0 */;
    ret = lsm6dso_write_reg(LSM6DSO_CTRL2_G, ctrl2_g);
    if (ret != 0) {
        return ret;
    }

    return 0;  /* Success */
}

/*----------------------------------------------------------------------
 * 7) Read accelerometer (raw counts), output in mg via sensitivity constant
 *----------------------------------------------------------------------*/
int lsm6dso_read_accel_mg(int16_t *out_x_mg, int16_t *out_y_mg, int16_t *out_z_mg) {
    int ret;
    uint8_t buf[6];

    /* Read 6 bytes starting from OUTX_L_XL (auto-increment is enabled) */
    ret = i2c_read_regs(LSM6DSO_I2C_ADDR, LSM6DSO_OUTX_L_XL, buf, 6);
    if (ret != 0) {
        return ret;
    }

    /* Combine and convert to mg (1 LSB = 0.061 mg for ±2 g) */
    int16_t raw_x = combine_bytes(buf[0], buf[1]);
    int16_t raw_y = combine_bytes(buf[2], buf[3]);
    int16_t raw_z = combine_bytes(buf[4], buf[5]);

    /* Multiply by sensitivity (0.061 mg/LSB). We return integer mg by rounding. */
    *out_x_mg = (int16_t)((float)raw_x * LSM6DSO_SENSITIVITY_ACCEL_2G + 0.5f);
    *out_y_mg = (int16_t)((float)raw_y * LSM6DSO_SENSITIVITY_ACCEL_2G + 0.5f);
    *out_z_mg = (int16_t)((float)raw_z * LSM6DSO_SENSITIVITY_ACCEL_2G + 0.5f);

    return 0;
}

/*----------------------------------------------------------------------
 * 8) Read gyroscope (raw counts), output in mdps via sensitivity constant
 *----------------------------------------------------------------------*/
int lsm6dso_read_gyro_mdps(int16_t *out_x_mdps, int16_t *out_y_mdps, int16_t *out_z_mdps) {
    int ret;
    uint8_t buf[6];

    /* Read 6 bytes starting from OUTX_L_G (auto-increment enabled) */
    ret = i2c_read_regs(LSM6DSO_I2C_ADDR, LSM6DSO_OUTX_L_G, buf, 6);
    if (ret != 0) {
        return ret;
    }

    /* Combine and convert to mdps (1 LSB = 8.75 mdps for 250 dps) */
    int16_t raw_x = combine_bytes(buf[0], buf[1]);
    int16_t raw_y = combine_bytes(buf[2], buf[3]);
    int16_t raw_z = combine_bytes(buf[4], buf[5]);

    /* Multiply by sensitivity (8.75 mdps/LSB). Return integer mdps by rounding. */
    *out_x_mdps = (int16_t)((float)raw_x * LSM6DSO_SENSITIVITY_GYRO_250DPS + 0.5f);
    *out_y_mdps = (int16_t)((float)raw_y * LSM6DSO_SENSITIVITY_GYRO_250DPS + 0.5f);
    *out_z_mdps = (int16_t)((float)raw_z * LSM6DSO_SENSITIVITY_GYRO_250DPS + 0.5f);

    return 0;
}




