#include "main.h"
#include <stm32f4xx.h> 
#include "usart6.h"

//void Referee_USART6_Configuration(u32 bound){
//		//GPIO�˿�����
//		GPIO_InitTypeDef GPIO_InitStructure;
//		USART_InitTypeDef USART_InitStructure;
//		NVIC_InitTypeDef NVIC_InitStructure;

//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //ʹ��GPIOGʱ��
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��

//		//����6��Ӧ���Ÿ���ӳ��
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //GPIOA9����ΪUSART6
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //GPIOA10����ΪUSART6

//		//USART6�˿�����
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9��GPIOG14
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //�ٶ�50MHz
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
//		GPIO_Init(GPIOG,&GPIO_InitStructure); //��ʼ��GPIOG9��GPIOG14

//		//USART6 ��ʼ������
//		USART_InitStructure.USART_BaudRate = bound;//����������
//		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;        //�շ�ģʽ
//		USART_Init(USART6, &USART_InitStructure); //��ʼ������6

//		USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���6 

//		USART_ClearFlag(USART6, USART_FLAG_TC);

//		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�

//		//USART6 NVIC ����
//		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����6�ж�ͨ��
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;                //�����ȼ�3
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                        //IRQͨ��ʹ��
//		NVIC_Init(&NVIC_InitStructure);        //����ָ���Ĳ�����ʼ��VIC�Ĵ�����
// 
//        
//}
u8 kzsb = 0;
void USART6_DMA_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	      GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
				NVIC_InitTypeDef NVIC_InitStructure;
        /* -------------- Enable Module Clock Source ----------------------------*/
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_DMA2,ENABLE); //ʹ��GPIOGʱ��
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��
				
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);

          //����6��Ӧ���Ÿ���ӳ��
				GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //G9����ΪUSART6
				GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //G14����ΪUSART6                                                        /* -------------- Configure GPIO ---------------------------------------*/
        {


						//USART6�˿�����
								GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9��GPIOG14
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;        //�ٶ�50MHz
								GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
								GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
								GPIO_Init(GPIOG,&GPIO_InitStructure); //��ʼ��GPIOG9��GPIOG14			
								//USART_DeInit(USART1);

                USART_InitStructure.USART_BaudRate = 115200;
                USART_InitStructure.USART_WordLength = USART_WordLength_8b;
                USART_InitStructure.USART_StopBits = USART_StopBits_1;
                USART_InitStructure.USART_Parity = USART_Parity_No;
                USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
                USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
                USART_Init(USART6, &USART_InitStructure);

                USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

                USART_ClearFlag(USART6, USART_FLAG_IDLE);
                USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

                USART_Cmd(USART6, ENABLE);
        }

        /* -------------- Configure NVIC ---------------------------------------*/
        {
                
                NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART6_NVIC;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
        }
				
        //DMA2 stream1 ch5     !!!!!!! P206 of the datasheet
        /* -------------- Configure DMA -----------------------------------------*/
        {
                DMA_InitTypeDef DMA_InitStructure;
                DMA_DeInit(DMA2_Stream1);

                DMA_InitStructure.DMA_Channel = DMA_Channel_5;
                DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
                DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;
                DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
                DMA_InitStructure.DMA_BufferSize = 512;
                DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
                DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
                DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
                DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
                DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
                DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
                DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
                DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
                DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
                DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
                DMA_Init(DMA2_Stream1, &DMA_InitStructure);
                DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)rx2_buf, DMA_Memory_0);
                DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
                DMA_Cmd(DMA2_Stream1, DISABLE); //Add a disable
                DMA_Cmd(DMA2_Stream1, ENABLE);
        }
}
