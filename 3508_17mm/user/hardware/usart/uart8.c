#include "uart8.h"
#include <string.h>
#include <stdarg.h>
#include "LX_IMU.h"

#define UART_REC_LEN  			8  	//�����������ֽ��� 200
#define EN_UART8_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����  
#define UART8_Waiting 0
#define UART8_Receiving 1
#define UART8_Success 2 
#define UART8_Failed  3

u8 Tx8Buffer[256];
u8 Tx8Counter = 0;
u8 count8 = 0;

void LX_IMU_UART8_Configuration(u32 bound)
{
		//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //ʹ��GPIOGʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8,ENABLE);//ʹ��UART8ʱ��

		//����8��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource0,GPIO_AF_UART8);
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource1,GPIO_AF_UART8);

		//UART8�˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
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
		USART_Init(UART8, &USART_InitStructure); //��ʼ������8

		USART_Cmd(UART8, ENABLE);  //ʹ�ܴ���8

		USART_ClearFlag(UART8, USART_FLAG_TC);

		USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);//��������ж�

		//UART8 NVIC ����
		NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;//����8�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART8_NVIC ;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                //�����ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                        //IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);        //����ָ���Ĳ�����ʼ��VIC�Ĵ�����

}

void UART8_IRQHandler(void)                        //����6�жϷ������
{		
		u8 com_data;

		if ( UART8->SR & USART_SR_ORE ) //ORE�ж�
    {
        com_data = UART5->DR;
    }		
    //�����ж�
    if ( USART_GetITStatus ( UART8, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART8, USART_IT_RXNE ); //����жϱ�־
				//
        com_data = UART8->DR;
				//
				ANO_DT_LX_Data_Receive_Prepare(com_data);
				Usart_SendByte(UART8,com_data);
			
    }
//    //���ͣ�������λ���ж�
//    if ( USART_GetITStatus ( UART8, USART_IT_TXE ) )
//    {
//        UART8->DR = Tx8Buffer[Tx8Counter++]; //дDR����жϱ�־
//        if ( Tx8Counter == count8 )
//        {
//            UART8->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
//        }
//    }		
		
} 

void Uart8_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        Tx8Buffer[count8++] = * ( DataToSend + i );
    }

    if ( ! ( UART8->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( UART8, USART_IT_TXE, ENABLE ); //�򿪷����ж�
    }

}

/*****************  ����һ���ֽ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** ����8λ������ ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* ����һ���ֽ����ݵ�USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* �ȴ�������� */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}
