#ifndef ALX_GYRO_ACCEL_H
#define ALX_GYRO_ACCEL_H

/* Change this if SA0 pin is tied differently.
 * If SA0 = 0 → 0x6A; if SA0 = 1 → 0x6B. */
#define LSM6DSO_I2C_ADDR 0x6B

extern int lsm6dso_init();
extern int lsm6dso_read_accel_mg(int16_t *out_x_mg, int16_t *out_y_mg, int16_t *out_z_mg);
extern int lsm6dso_read_gyro_mdps(int16_t *out_x_mdps, int16_t *out_y_mdps, int16_t *out_z_mdps);

#endif
