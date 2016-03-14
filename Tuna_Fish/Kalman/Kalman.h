#ifndef _KALMAN_H_
#define _KALMAN_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Basics.h"
#include "I2C_Basics.h"

bool complementry_init=false,kalman_init=false;

void init_complementry(float*);
void init_kalman(void);

void ComplementryFilter_YPR(float*,float*);
void KalmanFilter_YPR(float*,float*,float*,float*,float*,float*,float*);
	
#ifdef __cplusplus
}
#endif

#endif /* __BASICS_H */
