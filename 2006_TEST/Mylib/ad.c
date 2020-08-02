#include "main.h"

void Button_Detection_Configuration(void)
{
		ADC_InitTypeDef adc;
		GPIO_InitTypeDef gpio;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

		gpio.GPIO_Pin = GPIO_Pin_6;
		gpio.GPIO_Mode = GPIO_Mode_AN;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA,&gpio);

		adc.ADC_Resolution = ADC_Resolution_10b;
		adc.ADC_ScanConvMode = DISABLE;
		adc.ADC_ContinuousConvMode = ENABLE;
		adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
		adc.ADC_DataAlign = ADC_DataAlign_Right;
		adc.ADC_NbrOfConversion = 1;
		ADC_Init(ADC1,&adc);

		ADC_Cmd(ADC1,ENABLE);

		ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_56Cycles);
		
		Adc_Init();
}

//������ѹ���
unsigned int Button_Detect(void)
{
	  return Get_Adc(6);
}

//��ʼ��ADC
//����Ĭ�Ͻ�����ADC1_CH6																	   
void  Adc_Init(void)
{    
	//�ȳ�ʼ��IO��
 	RCC->APB2ENR|=1<<8;    	//ʹ��ADC1ʱ�� 
	RCC->AHB1ENR|=1<<0;    	//ʹ��PORTAʱ��	  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&gpio);  

	RCC->APB2RSTR|=1<<8;   	//ADCs��λ
	RCC->APB2RSTR&=~(1<<8);	//��λ����	 
	ADC->CCR=1<<16;			//ADCCLK=PCLK2/4=90/4=22.5Mhz,ADCʱ����ò�Ҫ����36Mhz
 	
	ADC1->CR1=0;   			//CR1��������
	ADC1->CR2=0;   			//CR2��������
	ADC1->CR1|=0<<24;      	//12λģʽ
	ADC1->CR1|=0<<8;    	//��ɨ��ģʽ	
	
	ADC1->CR2&=~(1<<1);    	//����ת��ģʽ
 	ADC1->CR2&=~(1<<11);   	//�Ҷ���	
	ADC1->CR2|=0<<28;    	//�������
	
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1|=0<<20;     	//1��ת���ڹ��������� Ҳ����ֻת����������1 			   
	//����ͨ��5�Ĳ���ʱ��
	ADC1->SMPR2&=~(7<<(3*5));//ͨ��5����ʱ�����	  
 	ADC1->SMPR2|=7<<(3*5); 	//ͨ��5  480������,��߲���ʱ�������߾�ȷ��	 
 	ADC1->CR2|=1<<0;	   	//����ADת����	  
}	
//���ADCֵ
//ch:ͨ��ֵ 0~18
//����ֵ:ת�����
u16 Get_Adc(u8 ch)   
{
	//����ת������	  		 
	ADC1->SQR3&=0XFFFFFFE0;//��������1 ͨ��ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<30;       //��������ת��ͨ�� 
	while(!(ADC1->SR&1<<1));//�ȴ�ת������	 	   
	return ADC1->DR;		//����adcֵ	
}
//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
}  