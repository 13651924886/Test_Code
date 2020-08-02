#include "main.h"
#include <stm32f4xx.h> 
 

//初始化IO 串口6 
//bound:波特率
void USART6_Configuration(u32 bound){
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOG时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟

		//串口6对应引脚复用映射
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //GPIOA9复用为USART6
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //GPIOA10复用为USART6

		//USART6端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9与GPIOG14
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化GPIOG9与GPIOG14

		//USART6 初始化设置
		USART_InitStructure.USART_BaudRate = bound;//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;        //收发模式
		USART_Init(USART6, &USART_InitStructure); //初始化串口6

		USART_Cmd(USART6, ENABLE);  //使能串口6 

		USART_ClearFlag(USART6, USART_FLAG_TC);

		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

		//USART6 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口6中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;                //子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                        //IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);        //根据指定的参数初始化VIC寄存器、
 
        
}


void USART6_IRQHandler(void)                        //串口6中断服务程序
{
		u8 Res;

		if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
				Res =USART_ReceiveData(USART6);//(USART6->DR);        //读取接收到的数据
				USART6_SendChar(Res);
							 
		} 
 
} 
      

void USART6_SendChar(u8 ch)
{
  while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
			USART6->DR = (u8) ch;   
}