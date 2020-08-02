/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

/* ������������ */
typedef float vec3_f[3];	//vec3_f a == float a[3]
typedef float vec2_f[2];
typedef s32 vec3_s32[3];
typedef s32 vec2_s32[2];
typedef s16 vec3_s16[3];
typedef s16 vec2_s16[2];

/* ��С�����ݷֽ� */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/* ��������ϵͳ��ʶ�� */
#define HW_ALL    0xFF
#define SWJ_ADDR 	0xAF
#define HW_TYPE	  0x61
#define HW_VER	  1
#define SOFT_VER  17
#define BL_VER	  0
#define PT_VER	  400
#define ANO_DT_USE_USART2 				//��������2��������
#define ANO_DT_USE_USB_HID				//�����ɿ�USBHID������λ������


//��̨�������can����ʧ�ܵ����������ʹ�� ����ӳٷ��Ϳ���ָ��ķ�ʽ���
#define GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE 0
//ѡ��ϵͳʱ��
#define SysCoreClock 180
//ѡ���ж����ȼ�
#define RC_NVIC 4
#define CAN1_NVIC 4
#define CAN2_NVIC 4
#define TIM3_NVIC 5
#define TIM6_NVIC 4
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5
#define PUSH_WHEEL_INT_NVIC 5
#define TRIGGER_WHEEL_INT_NVIC 5
#define UART8_NVIC 1	// ����IMU
#define USART6_NVIC 2	// ����ϵͳ�û��ӿ�
#define UART7_NVIC 3

//��Ҫ�����޸�
#define Latitude_At_ShenZhen 22.57025f

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif


#endif /* __MAIN_H */
