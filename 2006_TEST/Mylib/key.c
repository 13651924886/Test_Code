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
	static u8 key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(KEY==1))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY==1)return KEY_PRES;
	}else if(KEY==0)key_up=1; 	     
	return 0;// �ް�������
}
