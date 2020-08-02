#include "main.h"
#include "key.h"

void KEY_Configuration(void)
{
    GPIO_InitTypeDef  gpio;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
   
    gpio.GPIO_Pin = GPIO_Pin_2;   
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);
}

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY==1))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY==1)return KEY_PRES;
	}else if(KEY==0)key_up=1; 	     
	return 0;// 无按键按下
}
