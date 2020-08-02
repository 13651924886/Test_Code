#include "main.h"


/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

void CAN1_Configuration( void )
{
	CAN_InitTypeDef		can;
	CAN_FilterInitTypeDef	can_filter;
	GPIO_InitTypeDef	gpio;
	NVIC_InitTypeDef	nvic;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_CAN1, ENABLE );
	/* RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); */

	GPIO_PinAFConfig( GPIOD, GPIO_PinSource0, GPIO_AF_CAN1 );
	GPIO_PinAFConfig( GPIOD, GPIO_PinSource1, GPIO_AF_CAN1 );

	gpio.GPIO_Pin	= GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_Init( GPIOD, &gpio );

	nvic.NVIC_IRQChannel			= CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority	= 2;
	nvic.NVIC_IRQChannelSubPriority		= 1;
	nvic.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Init( &nvic );

	nvic.NVIC_IRQChannel			= CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority	= 0;
	nvic.NVIC_IRQChannelSubPriority		= 1;
	nvic.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Init( &nvic );


	CAN_DeInit( CAN1 );
	CAN_StructInit( &can );

	can.CAN_TTCM		= DISABLE;
	can.CAN_ABOM		= DISABLE;
	can.CAN_AWUM		= DISABLE;
	can.CAN_NART		= DISABLE;
	can.CAN_RFLM		= DISABLE;
	can.CAN_TXFP		= ENABLE;
	can.CAN_Mode		= CAN_Mode_Normal;
	can.CAN_SJW		= CAN_SJW_1tq;
	can.CAN_BS1		= CAN_BS1_7tq;
	can.CAN_BS2		= CAN_BS2_7tq;
	can.CAN_Prescaler	= 3; /* CAN BaudRate 45/(1+6+8)/3=1Mbps */
	CAN_Init( CAN1, &can );

	can_filter.CAN_FilterNumber		= 10;
	can_filter.CAN_FilterMode		= CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale		= CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh		= 0x0000;
	can_filter.CAN_FilterIdLow		= 0x0000;
	can_filter.CAN_FilterMaskIdHigh		= 0x0000;
	can_filter.CAN_FilterMaskIdLow		= 0x0000;
	can_filter.CAN_FilterFIFOAssignment	= 0;
	can_filter.CAN_FilterActivation		= ENABLE;
	CAN_FilterInit( &can_filter );
	CAN_ITConfig( CAN1, CAN_IT_FMP0, ENABLE );
	CAN_ITConfig( CAN1, CAN_IT_TME, ENABLE );
}


unsigned char can_tx_success_flag = 0;


/*************************************************************************
*                         CAN1_TX_IRQHandler
*  描述：CAN1的发送中断函数
*************************************************************************/
void CAN1_TX_IRQHandler( void )
{
	if ( CAN_GetITStatus( CAN1, CAN_IT_TME ) != RESET )
	{
		CAN_ClearITPendingBit( CAN1, CAN_IT_TME );
		can_tx_success_flag = 1;
	}
}


short	Velocity_Value_2006[1] = { 0 };
short	Position_Value_2006[1] = { 0 };
short	Torque_Value_2006[1] = { 0 };

short Velocity_2006_ID7=0;
short Position_2006_ID7=0;
short Torque_2006_ID7=0;

void CAN1_RX0_IRQHandler( void )
{
	CanRxMsg rx_message;
	if ( CAN_GetITStatus( CAN1, CAN_IT_FMP0 ) != RESET )
	{
		CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
		CAN_Receive( CAN1, CAN_FIFO0, &rx_message );

		if ( (rx_message.IDE == CAN_Id_Standard) && (rx_message.IDE == CAN_RTR_Data) && (rx_message.DLC == 8) ) /* 标准帧、数据帧、数据长度为8 */
		{
			if ( rx_message.StdId == 0x208 )                                                                /* 关于2006电机 */
			{
				Torque_Value_2006[0]	= (rx_message.Data[4] << 8) | (rx_message.Data[5]);
				Velocity_Value_2006[0]	= (rx_message.Data[2] << 8) | (rx_message.Data[3]);
				Position_Value_2006[0]	= (rx_message.Data[0] << 8) | (rx_message.Data[1]);
				
				Velocity_2006_ID7=Velocity_Value_2006[0];	
				Position_2006_ID7=Position_Value_2006[0];
				Torque_2006_ID7=Torque_Value_2006[0];
			}
		}
	}
}


