#ifndef __AD_H__
#define __AD_H__

void Button_Detection_Configuration(void);
unsigned int Button_Detect(void);

#define ADC_CH5  		6 		 	//ͨ��5	   	      	    
	   									   
void Adc_Init(void); 				//ADC��ʼ��
u16  Get_Adc(u8 ch); 				//���ĳ��ͨ��ֵ 
u16 Get_Adc_Average(u8 ch,u8 times);//�õ�ĳ��ͨ����������������ƽ��ֵ  

#endif
