#ifndef ALX_GYRO_ACCEL_H
#define ALX_GYRO_ACCEL_H

/* Change this if SA0 pin is tied differently.
 * If SA0 = 0 → 0x6A; if SA0 = 1 → 0x6B. */
#define LSM6DSO_I2C_ADDR 0x6B
#define LSM6DSO_WHO_AM_I_ID   0x6C
#define LSM6DS33_WHO_AM_I_ID  0x69

extern int lsm6dso_init(uint16_t *pVer);
extern int lsm6dso_read_accel_mg(int16_t *out_x_mg, int16_t *out_y_mg, int16_t *out_z_mg);
extern int lsm6dso_read_gyro_mdps(int16_t *out_x_mdps, int16_t *out_y_mdps, int16_t *out_z_mdps);

#endif
