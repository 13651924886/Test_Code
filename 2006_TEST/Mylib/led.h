#ifndef __LED_H__
#define __LED_H__

void Led_Configuration(void);

#define  LED_RED_ON()      GPIO_ResetBits(GPIOE, GPIO_Pin_11)
#define  LED_RED_OFF()     GPIO_SetBits(GPIOE, GPIO_Pin_11)
#define  LED_RED_TOGGLE()  GPIO_ToggleBits(GPIOE, GPIO_Pin_11)

#define  LED_GREEN_ON()        GPIO_ResetBits(GPIOF, GPIO_Pin_14)
#define  LED_GREEN_OFF()       GPIO_SetBits(GPIOF, GPIO_Pin_14)
#define  LED_GREEN_TOGGLE()    GPIO_ToggleBits(GPIOF, GPIO_Pin_14)

#endif
