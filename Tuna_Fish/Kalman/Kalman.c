#include "Kalman.h"
#include "Math.h"
/*
struct Raw_Data
{
	float AccX,AccY,AccZ;
	float GyroX,GyroY,GyroZ;
}	raw;
*/

struct Attitude
{
	float Yaw,Pitch,Roll;
}	Comp,Kalman;

void init_complementry(float *Accel_ptr)
{
	Comp.Pitch=0;
}

void ComplementryFilter_YPR(float Raw_Accel_Gyro[],float *YPR)
{
	if(complementry_init==false);
}

