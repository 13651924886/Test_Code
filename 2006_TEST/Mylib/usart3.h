#ifndef __USART3_H__
#define __USART3_H__
#include "main.h"
#define USART_REC_LEN  			8  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收  
#define UART3_Waiting 0
#define UART3_Receiving 1
#define UART3_Success 2 
#define UART3_Failed  3

void USART3_Configuration(void);
void USART3_SendChar(unsigned char b);
void UpdataData(void);
short DigitalCodeTrans(char Code);

typedef struct
{
	char Rx_Info[3];
	struct Float_{
	//float flag;
	float xspped_vel;
	float yspped_vel;
	float zspped_vel;

    }Float_xxx;
}Mainfold_Info;

//typedef union
//{
//unsigned char buffer1[8];
//float left_value;
//float right_value;
//}boost_data;
//extern boost_data buffer1;
extern Mainfold_Info Info_Mainfold ;


#endif
