/*Does most of Basic shit
-Blink LEDs
-UART Print
-Delay and Tics using System Tick TImer
By spookymelonhead <https://tachymoron.wordpress.com>
<brian.boozebacon@outlook.com>
*/

#include "basics.h"

extern GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;

#define SysTick_Reload 0xFFFFFF													//counts from 9000 to Zero..gives 0.001 which is 1ms
#define SysTick_Clock  0.0000001111111														// Configured it to be 1/72000000/8 in SysTick_Conf

/***********USART************/


void SerialDebug(unsigned int BaudRate)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);			 //Enable clock for USARt
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);		 	 //Enable GPIO clock
	
	GPIO_InitStructure.GPIO_Pin= (1<<9);			                //A9 USART1 TX
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF_PP;			        //Alternate function push pull
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;		
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);			                //Initialize
	GPIO_InitStructure.GPIO_Pin= (1<<10);			               //A10 USART1 RX
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN_FLOATING;		//Input mode and floating
	GPIO_Init(GPIOA,&GPIO_InitStructure);				     				//Intialize
	
	
	USART_InitStructure.USART_BaudRate= BaudRate;				           //Baudrate
	USART_InitStructure.USART_WordLength= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits= USART_StopBits_1;
	USART_InitStructure.USART_Parity= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx;			           //only tx for now
	USART_Init(USART1,&USART_InitStructure);				             //Initialize
	USART_Cmd(USART1,ENABLE);								                     //Enable USART
}

void PrintString(char *str)								//Prints strings
{
	while(*str !='\0')
	{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	USART_SendData(USART1,*str);
	str++;
	}
}

void PrintFloat(float val)					                        //prints floats
{
	char buffer[16];
	sprintf(buffer,"%f",val);
	PrintString(buffer);
	//unsigned char * b=(unsigned char*) (&val);			              //wasn't really working
	//for(uint8_t i=0; i<sizeof(float);USART_SendData(USART1,*(b+i)), i++);
}


/***********LEDs************/


void BeginBasics()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);		 //enable GPIO clock
	GPIO_InitStructure.GPIO_Pin= (1<<2) | (1<<3);		         //configure pins 2 and 3
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;			 //Ouput push pull
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;	         //set speed
	GPIO_Init(GPIOA,&GPIO_InitStructure);			          //initialize
	
	SysTick_Conf(SysTick_Reload);
}

void Blink()
{
	GPIOA->BSRR=(1<<3) | (1<<2);
	delay(50);
	GPIOA->BRR=(1<<3) | (1<<2);
	delay(50);
}



/***********BITSET BITCLEAR************/

uint8_t SetBit(uint32_t byte,uint8_t position)
{
	if(position>31) PrintString("\nBitset Error");
	else byte= byte | (1<<position);
	return byte;
}

uint8_t ClearBit(uint32_t byte,uint8_t position)
{
	if(position>31) PrintString("\nBitClear Error");
	else byte=byte & ~(1<<position);
	return byte;
}


/***********DELAY************/


void SysTick_Handler(void){tickle++;}
	//PrintString("\nSysTick Handler");


double tics()														//  
{
	return ((SysTick_Reload-SysTick->VAL) * SysTick_Clock + SysTick_Reload*SysTick_Clock*tickle);
}

void delay(unsigned int val)
{
	float current= tics();
	while(tics()-current<((float)val/1000));
}

bool SysTick_Conf(uint32_t ticks)
{
	if (ticks > 0x00FFFFFF){ PrintString("\nSysTick Err");  return (1);}          						 /* Reload value impossible */
                                                               
  SysTick->LOAD  = (ticks & 0x00FFFFFF)-1; 										 /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Cortex-M0 System Interrupts */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
	SysTick->CTRL=0;
	SysTick->CTRL  = SysTick_CTRL_TICKINT_Msk   | 
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
	PrintString("\nSysTick Configured");
	return (0);                                                  /* Function successful */
}

float dt()
{
	float donut;
	float spudnut=tics();
	float dT=(spudnut-donut);
	if(dT<0) PrintString("\nCongrats..! you're pity fucked.");
	donut=spudnut;
	return dT;
}
