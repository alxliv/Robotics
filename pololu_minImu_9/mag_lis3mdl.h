#ifndef MAG_LIS3MDL_H
#define MAG_LIS3MDL_H

#define LIS3MDL_I2C_ADDR 0x1E

extern int lis3mdl_init();
extern int lis3mdl_read(int16_t *out_x, int16_t *out_y, int16_t *out_z);

#endif
