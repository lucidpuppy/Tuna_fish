/*Does sorts of Basic shit
-Blink LEDs based on non timer based delay
-UART Print
By spookymelonhead <https://tachymoron.wordpress.com>
<brian.boozebacon@outlook.com>
*/

#ifndef _BASICS_H_
#define _BASICS_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"

#define True 1
#define False 0

void BeginBasics(void);
void Blink(void);
void SerialDebug(unsigned int);
void PrintString(char *);
void PrintFloat(float);

uint8_t SetBit(uint32_t,uint8_t);
uint8_t ClearBit(uint32_t,uint8_t);
static volatile uint32_t tickle;

void SysTick_Handler(void);
double tics(void);
bool SysTick_Conf(uint32_t);
void delay(unsigned int);

float dt(void);
#ifdef __cplusplus
}
#endif

#endif /* __BASICS_H */

