#include "main.h"

extern short Velocity_2006_ID7;
extern short Position_2006_ID7;
extern short Torque_2006_ID7;

unsigned char PF0_state=0;

#define POWER1_ON()  GPIO_SetBits(GPIOH, GPIO_Pin_2)
#define POWER1_OFF()  GPIO_ResetBits(GPIOH, GPIO_Pin_2)
#define POWER2_ON()  GPIO_SetBits(GPIOH, GPIO_Pin_3)
#define POWER2_OFF()  GPIO_ResetBits(GPIOH, GPIO_Pin_3)
#define POWER3_ON()  GPIO_SetBits(GPIOH, GPIO_Pin_4)
#define POWER3_OFF()  GPIO_ResetBits(GPIOH, GPIO_Pin_4)
#define POWER4_ON()  GPIO_SetBits(GPIOH, GPIO_Pin_5)
#define POWER4_OFF()  GPIO_ResetBits(GPIOH, GPIO_Pin_5)
#define PF0_1()  GPIO_SetBits(GPIOF, GPIO_Pin_0)
#define PF0_0()  GPIO_ResetBits(GPIOF, GPIO_Pin_0)
void Power_Configuration()
{
	GPIO_InitTypeDef gpioInitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_2;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOH, &gpioInitStruct);
	GPIO_ResetBits(GPIOH, GPIO_Pin_2);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_3;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOH, &gpioInitStruct);
	GPIO_ResetBits(GPIOH, GPIO_Pin_3);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_4;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOH, &gpioInitStruct);
	GPIO_ResetBits(GPIOH, GPIO_Pin_4);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_5;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOH, &gpioInitStruct);
	GPIO_ResetBits(GPIOH, GPIO_Pin_5);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_0;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &gpioInitStruct);
	GPIO_ResetBits(GPIOF, GPIO_Pin_0);
}

extern unsigned char can_tx_success_flag ;
void SetM2006(short ID5 ,short ID6, short ID7, short ID8)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    can_id = 0x1FF; 
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = ( ID5 >> 8 ) ;
    tx_message.Data[1] = ID5;//ID 5
    tx_message.Data[2] =  ( ID6 >> 8 ) ;
    tx_message.Data[3] = ID6;//ID 6
    tx_message.Data[4] = ( ID7 >> 8 ) ;
    tx_message.Data[5] = ID7;//ID 7
    tx_message.Data[6] = ( ID8 >> 8 ) ;
    tx_message.Data[7] = ID8;//ID 8
    
    //can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    //while(can_tx_success_flag == 0);
}

struct _pid{
    short SetSpeed;           //?????
    short ActualSpeed;        //?????
    short err;               //?????
    short err_last;           //????????
    float Kp,Ki,Kd;           //????????????
    short voltage;         //?????(????????)
    short integral;           //?????
	  short voltage_high_limit,voltage_low_limit;
}pid;

void PID_init(){
    printf("PID_init begin \n");
    pid.SetSpeed=0;
    pid.ActualSpeed=0;
    pid.err=0;
    pid.err_last=0;
    pid.voltage=0;
		pid.voltage_high_limit=10000;
	  pid.voltage_low_limit=-10000;
    pid.integral=0;
    pid.Kp=4.0;
    pid.Ki=0.5;
    pid.Kd=0.1;
    printf("PID_init end \n");
}

short PID_realize(short set_speed,short act_peed){
    pid.SetSpeed=set_speed;
	  pid.ActualSpeed=act_peed;
	
    pid.err=pid.SetSpeed-pid.ActualSpeed;
    pid.integral+=pid.err;
    pid.voltage=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
    pid.err_last=pid.err;
	
	  if(pid.voltage<=pid.voltage_high_limit && pid.voltage>=pid.voltage_low_limit)
			return pid.voltage;
		else if(pid.voltage>pid.voltage_high_limit)
			return pid.voltage_high_limit;
		else if(pid.voltage<pid.voltage_low_limit)
			return pid.voltage_low_limit;
		else
			return 0;
}


int main( void )
{
	unsigned int rc_ch0;
	delay_init( 180 );      /* 延时函数初始化 */
	KEY_Configuration();
	Led_Configuration();
	
	USART1_Configuration(); /* 遥控器接收机信息 板载9号接口 接插时注意方向！ */

	USART3_Configuration(); /* 串口打印用 USART3（波特率115200） 也就是板载的19号接口 使用溪地的线 则黑色为GND 黄色为RXD 蓝色为TXD */
  printf( "USART3_Configuration END\r\n" );
	
	USART6_Configuration( 115200 );
	
	Button_Detection_Configuration();
	Power_Configuration();
	POWER1_ON();
	POWER2_ON();
	POWER3_ON();
	POWER4_ON();
	CAN1_Configuration();
	delay_ms(100);
	
	PID_init();  
	
	Tick_TIM7_Init( 1000 );  /* 间隔为10ms */
	while (1)
	{
			printf( "Velocity_2006_ID7:%d\r\n",Velocity_2006_ID7);
		  delay_ms(10);
	}
}

void TIM7_IRQHandler( void ) /* 间隔为10ms */
{
	if ( TIM_GetITStatus( TIM7, TIM_IT_Update ) == SET )
	{
		short set_current=PID_realize(500,Velocity_2006_ID7);
		if(PF0_state==0)
		{
			PF0_1();
			PF0_state=1;
		}
		else
		{
			PF0_0();
			PF0_state=0;
		}
		SetM2006(0,0,0,set_current);
	}
	TIM_ClearITPendingBit( TIM7, TIM_IT_Update );
}