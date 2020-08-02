#ifndef __USART1_H__
#define __USART1_H__

#include <stm32f4xx.h>
#include <stdio.h>

#define RC_S1Up 1
#define RC_S1Down 2
#define RC_S1Middle 3

#define RC_S2Up 1
#define RC_S2Down 2
#define RC_S2Middle 3

typedef struct
{
	struct
	{ 
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned char s1;
		unsigned char s2;
	}rc;
	
	struct 
	{
		unsigned short x;
		unsigned short y;
		unsigned short z;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	
	struct
	{
		unsigned short v;
	}key;
}RC_Ctl_t;

extern volatile  RC_Ctl_t RC_Ctl;


void USART1_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void USART1_Configuration(void);
#endif
