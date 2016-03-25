#include "Basics.h"
#include "I2C_basics.h"
#include "MPU6050.h"
#include "Math.h"
#include "Kalman.h"
#include "Quaternions.h"

#define GyroX_offset 13														     //Gyro Offsets Register from 0x13 to 0x18
#define GyroY_offset -25
#define GyroZ_offset -8

#define AccelX_offset 1053														//Accel Offsets Register from 0x06 to 0x0B
#define AccelY_offset -1697
#define AccelZ_offset 1575

volatile unsigned int *DWT_CTRL= 	(volatile unsigned int *) 0xE0001000;
volatile unsigned int *DWT_CYCCNT=  (volatile unsigned int *) 0xE0001004;
volatile unsigned int *SCB_DEMCR= 	(volatile unsigned int *) 0xE000EDFC;

GPIO_InitTypeDef GPIO_InitStructure;
I2C_InitTypeDef I2C_InitStructure;

void Enable_PeriphClock(void);
int16_t MPU6050_Offsets[6]={GyroX_offset,GyroY_offset,GyroZ_offset,AccelX_offset,AccelY_offset,AccelZ_offset};

float spudnut,donut;
float delt;

void Display_Raw(float accel[3],float gyro[3],float temp);
void DisplayVector(float vector[3]);

bool confirm_offsets(void);

void RemoveGravity(float rpy_k[3],float accel[3],float accel_rw[3]);

int main()
{
	SerialDebug(250000);			//PrintString() and PrintFloat() using UART
	BeginBasics();
	Blink();
	Enable_PeriphClock();
	
	/*Roll-0 Pitch-1 Yaw-2..Roll rotation around X axis.Pitch rotation around Y axis and Yaw rotation around Z axis
	note: It does not mean rotation along the X Y or Z axis*/
	
	float RPY_c[3],RPY_k[3];					//RPY_c and RPY_k..Roll pitch and yaw obtained from complemntary filter and kalman filter													
	float Accel[3],Gyro[3],Tempreature;			//raw values
	float Accel_RealWorld[3];					//Real World Acceleration

	while(Init_I2C(400)){PrintString("\nI2C Connection Error");}	
	while(MPU6050_Init()){PrintString("MPU6050 Initialization Error");}
	MPU6050_UpdateOffsets(&MPU6050_Offsets[0]);
	//MPU6050_ConfirmOffsets(&MPU6050_Offsets[0]);
	
	while(0)						//to play around with quaternions and vector rotation
	{
		float vector[3]= {1,0,0};
		float axis_vector[3]= {0,1,0};		//rotate around Y axis
		float rot_angle=90;					//with 90 degrees
		
		Quaternion q;
		
		q=RotateVectorY(vector,rot_angle);	//Rotates vector around Y axis with rot_angle

		PrintString("\nRotated Vector's Quaternion\t");
		DisplayQ(q);
		
		PrintString("\n");
	}
	
	while(1)
	{
		MPU6050_GetRaw(&Accel[0],&Gyro[0],&Tempreature);	//Reads MPU6050 Raw Data Buffer..i.e Accel Gyro and Tempreature values

		if(Gyro[2]<0.3 && Gyro[2]>-0.3) Gyro[2]=0;			//this actually reduces Yaw drift..will add magnetometer soon

		spudnut=tics();										//tics() return current timing info..using SysTick running at CPU_Core_Frequency/8..Counter Runs from 0xFFFFFF to 0 therefore overflows every 1.864135 secs
		delt=spudnut-donut;									//small time dt
		donut=spudnut;											
		
		//Display_Raw(Accel,Gyro,Tempreature);
		
		Attitude_k(Accel,Gyro,RPY_k,delt);	//Estimates YPR using Kalman
		
		
		PrintString("\nYPR\t");
		PrintFloat(RPY_k[2]);
		PrintString("\t");
		PrintFloat(RPY_k[1]);
		PrintString("\t");
		PrintFloat(RPY_k[0]);
		
		RemoveGravity(RPY_k,Accel,Accel_RealWorld);
		PrintString("\tReal World Accel with Gravity\t");			//still glitchish..working on it
		DisplayVector(Accel_RealWorld);
		
		PrintString("\t");
		PrintFloat(1/delt);
	}
}

void Enable_PeriphClock(void)											//UART and LEDs are Enabled by default by Basics.c..rest we got to enable here.
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);		 	 //Enable GPIO clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);				 //Enable I2C1
}

void RemoveGravity(float rpy_k[3],float accel[3],float accel_rw[3])
{
	Quaternion Q=RotateVector3D(accel,rpy_k[2],rpy_k[1],rpy_k[0]);
	accel_rw[0]=Q.axis[0];
	accel_rw[1]=Q.axis[1];
	accel_rw[2]=Q.axis[2];						//Subtract 1 to remove gravity, still not good ..with yaw movements..*working on it
}

void Display_Raw(float accel[3], float gyro[3], float temp)		//Display raw Accel Gyro and Tempreature Values
{
	PrintString("\nAccel\tTemp\tGyro\t");
	
	for(uint8_t i=0;i<3;i++)
	{
		PrintFloat(accel[i]);
		PrintString("\t");
	}
	
	PrintFloat(temp);
	PrintString("\t");
	
	for(uint8_t i=0;i<3;i++)
	{
		PrintFloat(gyro[i]);
		PrintString("\t");
	}
}

void DisplayVector(float vector[3])				//Display array with 3 elements
{
	PrintString("<");
	PrintFloat(vector[0]);
	PrintString(",");
	PrintFloat(vector[1]);
	PrintString(",");
	PrintFloat(vector[2]);
	PrintString(">");
}

void Enable_DWT()
{
	bool DWT_en=false;
	if(!DWT_en)
	{
		*SCB_DEMCR =  *SCB_DEMCR | 0x01000000;
    *DWT_CYCCNT = 0; 															// reset the counter
    *DWT_CTRL =   *DWT_CTRL | 1 ; 								// enable the counter
	}
}

