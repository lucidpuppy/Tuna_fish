/*Does Basic I2C shit
-Read Write Register
-Read Write Register bits
-Read Write Registers Burst
By spookymelonhead <https://tachymoron.wordpress.com>
<brian.boozebacon@outlook.com>
*/

#include "i2c_basics.h"
#include "stm32f10x_i2c.h"

extern void PrintString(char*);
extern void Blink(void);
extern void delay(uint32_t);
extern uint8_t SetBit(uint32_t,uint8_t);
extern uint8_t ClearBit(uint32_t,uint8_t);

extern GPIO_InitTypeDef GPIO_InitStructure;
extern I2C_InitTypeDef I2C_InitStructure;												//Clock and InitStructure object to be enbled and defined in main.


bool Init_I2C(uint16_t clk_speed)
{
	if(clk_speed>400) return 1;
	
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_10MHz;
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	PrintString("\nGPIO PortB pins 6 SCL & 7 SDA configured for Open Drain");
	Blink();
	
	I2C_Cmd(I2C1,ENABLE);
	
	I2C_InitStructure.I2C_Ack= I2C_Ack_Enable;
	I2C_InitStructure.I2C_ClockSpeed= clk_speed * 1000;
	I2C_InitStructure.I2C_AcknowledgedAddress= I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_DutyCycle= I2C_DutyCycle_16_9;
	I2C_InitStructure.I2C_Mode= I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1= 0x00;
	
	I2C_Init(I2C1,&I2C_InitStructure);
	PrintString("\nI2C Configured with Duty Cycle 2 and Ack Enable");
	Blink();
	return 0;
}

uint8_t I2C_RegisterRead(uint8_t dev_addr, uint8_t addr)
{
	dev_addr=dev_addr<<1;
	
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1,dev_addr,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	I2C_SendData(I2C1,addr);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1,dev_addr,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	I2C_AcknowledgeConfig(I2C1,DISABLE);
	
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	uint8_t byte= I2C_ReceiveData(I2C1);
	
	I2C_AcknowledgeConfig(I2C1,ENABLE);
	
	I2C_GenerateSTOP(I2C1,ENABLE);
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_STOPF));
	
	return byte;
}

bool I2C_RegisterReadBit(uint8_t dev_addr, uint8_t addr, uint8_t pos)
{
	uint8_t byte =I2C_RegisterRead(dev_addr,addr);
	bool bit= (byte>>pos) & 0x01;
	return bit;
}

void I2C_RegisterReadBurst(uint8_t dev_addr, uint8_t addr,uint8_t n_bytes,uint8_t *ptr)
{
	dev_addr=dev_addr<<1;
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1,dev_addr,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	I2C_SendData(I2C1,addr);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1,dev_addr,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	while(n_bytes)
	{
		if(n_bytes==1)
		{
			I2C_AcknowledgeConfig(I2C1,DISABLE);
			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
			*ptr= I2C_ReceiveData(I2C1);
			I2C_AcknowledgeConfig(I2C1,ENABLE);
			n_bytes--;
		}
		else
		{
			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
			*ptr= I2C_ReceiveData(I2C1);
			ptr++;
			n_bytes--;
		}
	}
	
	I2C_GenerateSTOP(I2C1,ENABLE);
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_STOPF));
}

void I2C_RegisterWrite(uint8_t dev_addr,uint8_t addr,uint8_t data)
{
	dev_addr=dev_addr<<1;
	
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1,dev_addr,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	I2C_SendData(I2C1,addr);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_SendData(I2C1,data);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_GenerateSTOP(I2C1,ENABLE);
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_STOPF));
}

void I2C_RegisterWriteBit(uint8_t dev_addr, uint8_t addr, uint8_t position, bool val)
{
	uint8_t byte= I2C_RegisterRead(dev_addr,addr);
	
	if(val==1) byte=SetBit(byte,position);
	else byte=ClearBit(byte,position);
	I2C_RegisterWrite(dev_addr,addr,byte);
}

void I2C_RegisterWriteBurst(uint8_t dev_addr, uint8_t addr,uint8_t n_bytes,uint8_t *ptr)
{
	dev_addr=dev_addr<<1;
	
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1,dev_addr,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	I2C_SendData(I2C1,addr);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	while(n_bytes)
	{
		I2C_SendData(I2C1,*ptr);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		n_bytes--;
		ptr++;
		PrintString("\nYee");
	}
	
	I2C_GenerateSTOP(I2C1,ENABLE);
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_STOPF));
}	
