#ifndef ALL_CALIBRATE_MAGNETO
#define ALL_CALIBRATE_MAGNETO

extern void load_magneto_calibration(uint16_t ver);
extern void magnetometer_apply_calibration(int16_t mx_raw, int16_t my_raw, int16_t mz_raw,
                                    float *mx_cal, float *my_cal, float *mz_cal);
extern void calibrate_magneto();

#endif
