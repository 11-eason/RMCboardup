#ifndef BMI088_SIMPLE_H
#define BMI088_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

int bmi088_init(void);

int bmi088_read(float acceleration_mps2[3], float angular_velocity_deg_s[3]);

#ifdef __cplusplus
}
#endif

#endif
