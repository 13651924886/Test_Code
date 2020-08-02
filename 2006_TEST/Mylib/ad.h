#ifndef __AD_H__
#define __AD_H__

void Button_Detection_Configuration(void);
unsigned int Button_Detect(void);

#define ADC_CH5  		6 		 	//通道5	   	      	    
	   									   
void Adc_Init(void); 				//ADC初始化
u16  Get_Adc(u8 ch); 				//获得某个通道值 
u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道给定次数采样的平均值  

#endif
