#include "MPU6050.h"

bool MPU6050_CheckConnection()
{
	if(I2C_RegisterRead(0x68,0x75)==0x68) return 1;
	else return 0;
}

bool MPU6050_Init()
{
	if(I2C_RegisterRead(0x68,0x75)!=0x68) return 1;
	
	
	I2C_RegisterWrite(0x68,0x6B,0x02);		//Disable Sleep. Clock Source PLL with Gyro X reference.
	I2C_RegisterWrite(0x68,0x1A,0x00);
	I2C_RegisterWrite(0x68,0x1B,0x00);
	I2C_RegisterWrite(0x68,0x1C,0x00);
	I2C_RegisterWrite(0x68,0x23,0xFF);
	I2C_RegisterWrite(0x68,0x19,0x07);		//Sample Rate Divider
	PrintString("\nMPU6050 ready");
	return 0;
}

bool MPU6050_UpdateOffsets(int16_t *ptr)					//Writes Offsets to internal registers
{
	for(uint8_t i=0;i<3;i++)
	{	
		I2C_RegisterWrite(0x68,GyroOffsets_BaseAddress + 2*i ,(uint8_t) ((*ptr>>8)&0xff));
		I2C_RegisterWrite(0x68,GyroOffsets_BaseAddress + 2*i + 1,(uint8_t)((*ptr)&0xff));
		ptr++;
	}
	
	for(uint8_t i=0;i<3;i++)
	{	
		I2C_RegisterWrite(0x68,AccelOffsets_BaseAddress + 2*i ,(uint8_t) ((*ptr>>8)&0xff));
		I2C_RegisterWrite(0x68,AccelOffsets_BaseAddress + 2*i + 1,(uint8_t)((*ptr)&0xff));
		ptr++;
	}
	
	PrintString("\nOffsets Updated");
	
	uint8_t offsets[12];
	I2C_RegisterReadBurst(0x68,0x13,12,&offsets[0]);
	I2C_RegisterReadBurst(0x68,0x06,12,&offsets[6]);
	
	return(1);
}


void MPU6050_SoftReset(void)					//Soft Reset 7th bit PWR_MGMT_1
{
	I2C_RegisterWriteBit(0x68,0x6b,7,0);
}

void MPU6050_GetRaw(float *aptr,float *gptr,float *temp)
{
	uint8_t buffer[14];
	
	I2C_RegisterReadBurst(0x68,0x3B,14,&buffer[0]);
	
	/*
	*ptr= (float) ((int16_t)(buffer[0]<<8 | buffer[1])); ptr++;
	*ptr= (float) ((int16_t)(buffer[2]<<8 | buffer[3])); ptr++;
	*ptr= (float) ((int16_t)(buffer[4]<<8 | buffer[5])); ptr++;
	*ptr= (float) ((int16_t)(buffer[6]<<8 | buffer[7])); ptr++;
	*ptr= (float) ((int16_t)(buffer[8]<<8 | buffer[9])); ptr++;
	*ptr= (float) ((int16_t)(buffer[10]<<8 | buffer[11])); ptr++;
	*ptr= (float) ((int16_t)(buffer[12]<<8 | buffer[13]));
	*/
	
	for(uint8_t i=0; i<3;i++)
	{
		*aptr= (float) ((int16_t)(buffer[2*i]<<8 | buffer[(2*i)+1]))/16384;
		aptr++;
	}
	
	*temp= (float)((int16_t)(buffer[6]<<8 | buffer[7]) + 12412)/340;
	
	for(uint8_t i=4; i<7;i++)
	{
		*gptr= (float) ((int16_t)(buffer[2*i]<<8 | buffer[(2*i)+1]))/131;
		gptr++;
	}
}

void MPU6050_Buffer(uint8_t *ptr ,uint8_t n_bytes)
{
	PrintString("\nBuffer\t");
	for(uint8_t i=0;i<n_bytes;i++)
	{
		PrintFloat(*ptr);
		PrintString("   ");
		ptr++;
	}
}

bool MPU6050_ConfirmOffsets(int16_t *ptr)
{
	uint8_t offsets[12],count=0;
	
	I2C_RegisterReadBurst(0x68,0x13,6,&offsets[0]);
	I2C_RegisterReadBurst(0x68,0x06,6,&offsets[6]);
	
	PrintString("\t");
	for(uint8_t i=0;i<6;i++)
	{
		if((int16_t)(offsets[2*i] <<8 |offsets[2*i+1])==*ptr){
			count++; ptr++;}
		else {
			PrintString("\nOffset mismatch\t"); PrintFloat((int16_t)(offsets[2*i] <<8 |offsets[2*i+1])); return 0;}
	}
	return 1;
}
