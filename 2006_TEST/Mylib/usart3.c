#include "main.h"
#include <stm32f4xx.h>
int USART_RX_STA=0;
Mainfold_Info Info_Mainfold;
#define BeginFlag  0xff
#define EndFlag    0x80

#define PC 0
#define Mainfold 1
#define ticks_meter 123077.0   //???????   linear = 2.6
#define base_width 0.40f;      //??  angule = 0.316
#define robot_timer 0.53f      //??

struct Encoder_{
    
    float d_left;
    float d_right;int enc_left;        //wheel encoder readings
    int enc_right;
    int left;          // actual values coming back from robot
    int right;
}self;


union Max_Value
{
    unsigned char buf[20];
    struct _Float_{
       float hander;
       float _float_vx;
			 float _float_vy;
       float _float_vth;
			 float _float_v_th;
    }Float_RAM;
}Send_Data;


void UpdataData(void)
{	
    //for(i=0;i<20;i++)
    //USART3_SendChar(Send_Data.buf[i]);

}

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/
    
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目

void USART3_Configuration(void)
{
		USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
		NVIC_InitTypeDef  nvic;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

		GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 

		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD,&gpio);

		usart3.USART_BaudRate = 115200;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

		USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
		USART_Cmd(USART3,ENABLE);

		nvic.NVIC_IRQChannel =USART3_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority =4;
		nvic.NVIC_IRQChannelSubPriority =4;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
}

void USART3_SendChar(unsigned char b)
{
		while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
		USART_SendData(USART3,b);
}

int fputc(int ch, FILE *f)
{
	USART_SendData(USART3, (uint8_t)ch);
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    return ch;
}

char Rx_data = 0;

u8 UART3_flag = 0;

void USART3_IRQHandler(void)      //缺陷  如底盘控制 ff 00 05 05 05 80与  ff 00 05 05 05 00 80没有区别   \\按照现在的算法，都是一旦给定，如果不改变给定值，则一直是有效的。
{
	u8 Res;
  u8 j=0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{   
		Res=USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
		
		if(Res ==0xff  )//如错误发送0xff  0x50  然后发送正常的数据。会引起不正常
		{
			USART_RX_STA = 0;
			UART3_flag = 1;
			  //可以取消掉，在成功后动作处置零
			return;
		}
		 
	}
} 

