/*MPU6050
-Read RAW data buffer
-Updates Offsets
By spookymelonhead <https://tachymoron.wordpress.com>
<brian.boozebacon@outlook.com>
*/

#ifndef _MPU6050_H_
#define _MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"
#include "basics.h"
	
#define GyroOffsets_BaseAddress 0x13
#define AccelOffsets_BaseAddress 0x06
	
	
bool MPU6050_CheckConnection(void);
bool MPU6050_Init(void);
bool MPU6050_UpdateOffsets(int16_t *);				//Writes Offsets to internal registers
void MPU6050_SoftReset(void);					//Soft Reset 7th bit PWR_MGMT_1
void MPU6050_GetRaw(float *,float *,float *);					//Display Raw Values
void MPU6050_Buffer(uint8_t * ptr,uint8_t n_bytes);			//Display Buffer
void MPU6050_ArrangePackets(float *,float);				//Arrange Packets
bool MPU6050_ConfirmOffsets(int16_t*);
	

extern uint8_t I2C_RegisterRead(uint8_t,uint8_t);
extern bool I2C_RegisterReadBit(uint8_t, uint8_t, uint8_t);
extern void I2C_RegisterReadBurst(uint8_t,uint8_t,uint8_t,uint8_t *);
extern void I2C_RegisterWrite(uint8_t,uint8_t,uint8_t);									//Write register
extern void I2C_RegisterWriteBit(uint8_t,uint8_t,uint8_t,bool);
extern void I2C_RegisterWriteBurst(uint8_t,uint8_t,uint8_t,uint8_t*);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */

