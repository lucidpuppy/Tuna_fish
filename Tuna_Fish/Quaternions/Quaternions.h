#ifndef _QUATERNIONS_H_
#define _QUATERNIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Basics.h"
#include "I2C_Basics.h"

typedef struct
{
	float w,axis[3];							//w = sin(theta/2) and axis[3] equals axis vector * sin(theta/2)
} Quaternion;


Quaternion CreateQ(float axis_vector[3], float rot_angle);
Quaternion NormalizeQ(Quaternion q);
Quaternion MultiplyQ(Quaternion,Quaternion);					 //Multiply's two quaternions and returns product
Quaternion AddQ(Quaternion,Quaternion);							   //Adds two quaternions and returns sum
Quaternion QInverse(Quaternion);										   //inverse of a quaternion
void DisplayQ(Quaternion);
Quaternion RotateVector(float vector[3],float axis[3],float rot_angle);
Quaternion RotateVectorX(float vector[3],float rot_angle);
Quaternion RotateVectorY(float vector[3],float rot_angle);
Quaternion RotateVectorZ(float vector[3],float rot_angle);
Quaternion RotateVector3D(float vector[3],float yaw,float pitch,float roll);

#ifdef __cplusplus
}
#endif

#endif /* __BASICS_H */
