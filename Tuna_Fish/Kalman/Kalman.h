#ifndef _KALMAN_H_
#define _KALMAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Basics.h"
#include "I2C_Basics.h"


void Attitude_c(float accel[3],float gyro[3], float rpy_c[3],float delt);
void Attitude_k(float accel[3],float gyro[3],float rpy_k[3],float delt);

#ifdef __cplusplus
}
#endif

#endif /* __BASICS_H */
