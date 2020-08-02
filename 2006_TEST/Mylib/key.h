#ifndef __KEY_H__
#define __KEY_H__

#define KEY  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)//读取按键0
 

#define KEY_PRES	1		//KEY  

void KEY_Configuration(void);//IO初始化
u8 KEY_Scan(u8 mode);  	//按键扫描函数		

#endif
