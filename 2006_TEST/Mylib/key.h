#ifndef __KEY_H__
#define __KEY_H__

#define KEY  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)//��ȡ����0
 

#define KEY_PRES	1		//KEY  

void KEY_Configuration(void);//IO��ʼ��
u8 KEY_Scan(u8 mode);  	//����ɨ�躯��		

#endif
