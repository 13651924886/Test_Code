#include "uart7.h"

u8 TxBuffer[256];
u8 TxCounter = 0;
u8 count = 0;

u8 Rx_Buf[256];	//���ڽ��ջ���

void ANO_DT_UART7_Configuration(u32 bound)
{
		//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //ʹ��GPIOEʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//ʹ��UART7ʱ��

		//����8��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART8);
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART8);

		//UART8�˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //�ٶ�50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOE,&GPIO_InitStructure); 

		//UART8 ��ʼ������
		USART_InitStructure.USART_BaudRate = bound;//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;        //�շ�ģʽ
		USART_Init(UART7, &USART_InitStructure); //��ʼ������7

		USART_Cmd(UART7, ENABLE);  //ʹ�ܴ���7

		USART_ClearFlag(UART7, USART_FLAG_TC);

		USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);//��������ж�

		//UART8 NVIC ����
		NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//����7�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART7_NVIC ;//��ռ���ȼ�
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                //�����ȼ�
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                        //IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);        //����ָ���Ĳ�����ʼ��VIC�Ĵ�����

}


void Uart7_IRQ ( void )
{
    u8 com_data;

    if ( UART7->SR & USART_SR_ORE ) //ORE�ж�
    {
        //com_data = USART2->DR;
			USART_ReceiveData(UART7);
    }

    //�����ж�
    if ( USART_GetITStatus ( UART7, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART7, USART_IT_RXNE ); //����жϱ�־

        com_data = UART7->DR;
        //ANO_DT_Data_Receive_Prepare ( com_data );
				
			//UART7_Put_Char(com_data);//����ʽ֡
				//ANODT_GetByte(com_data);	//������д�뷵��
		}
    //���ͣ�������λ���ж�
    if ( USART_GetITStatus ( UART7, USART_IT_TXE ) )
    {

        UART7->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־
        if ( TxCounter == count )
        {
            UART7->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }



}
static void UART7_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;
	USART_ITConfig(UART7, USART_IT_TXE, ENABLE);
}

void UART7_Put_Buf( unsigned char *DataToSend, u8 data_num )
{
	for(u8 i = 0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}
	if(!(UART7->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(UART7, USART_IT_TXE, ENABLE);
}

//void UART7_Send( unsigned char *DataToSend , u8 data_num )
//{
//    u8 i;
//    for ( i = 0; i < data_num; i++ )
//    {
//        TxBuffer[count++] = * ( DataToSend + i );
//    }

//    if ( ! ( UART7->CR1 & USART_CR1_TXEIE ) )
//    {
//        USART_ITConfig ( UART7, USART_IT_TXE, ENABLE ); //�򿪷����ж�
//    }

//}
void UART7_IRQHandler(void)                        //����7�жϷ������
{
		Uart7_IRQ();
}
