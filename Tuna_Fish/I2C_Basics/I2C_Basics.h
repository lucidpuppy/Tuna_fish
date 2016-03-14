/*Does Basic I2C shit
-Read Write Register
-Read Write Register bits
-Read Registers Burst
By spookymelonhead <https://tachymoron.wordpress.com>
<brian.boozebacon@outlook.com>
*/

#ifndef _I2C_BASICS_H_
#define _I2C_BASICS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"

bool Init_I2C(uint16_t);																					//Initialize I2C

uint8_t I2C_RegisterRead(uint8_t,uint8_t);												//Read Register..device address,register address, how many bytes to read
bool I2C_RegisterReadBit(uint8_t, uint8_t, uint8_t);
void I2C_RegisterReadBurst(uint8_t,uint8_t,uint8_t,uint8_t *);
	
void I2C_RegisterWrite(uint8_t,uint8_t,uint8_t);									//Write register
void I2C_RegisterWriteBit(uint8_t,uint8_t,uint8_t,bool);
void I2C_RegisterWriteBurst(uint8_t,uint8_t,uint8_t,uint8_t*);


#ifdef __cplusplus
}
#endif

#endif /* __I2C_BASICS_H */

