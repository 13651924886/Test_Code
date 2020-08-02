/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��h�ļ��������ȫ�ֺ궨���Լ�
  *             typedef һЩ������������
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
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"
#include "usart6.h"
#include "uart8.h"
#include "GM6020_pwm.h"

#include "exit_init.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "calibrate_task.h"
#include "remote_control.h"
#include "start_task.h"
#include "referee_usart_task.h"
#include "user_task.h"
#include "LX_IMU.h"
#include "Ano_DT.h"

#define configTICK_RATE_HZ 1000

void BSP_init(void);

int main(void)
{
		BSP_init(); //Board Support Package  Ӳ����ʼ����GPIO\DMA\)
    delay_ms(100);// ��֤ǰ������
		Scheduler_Setup();// �������������ʼ�������������̣߳�������������
    while (1)
    {
       Scheduler_Run();			//�������������������ϵͳ���ܣ������жϷ������������������������� ;
    }
}

//�ĸ�24v ��� ���ο��� ��� 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //�ж��� 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //��ʼ���δ�ʱ�� 1ms SysTick�ж�  �жϷ�����λ��delay.c
    delay_init(configTICK_RATE_HZ);
    //��ˮ�ƣ����̵Ƴ�ʼ��
    led_configuration();
    //24������ƿ� ��ʼ��
    power_ctrl_configuration();
    //Ħ���ֵ��PWM��ʼ�� TIM8 CH3 CH4
    fric_PWM_configuration();
    //����IO��ʼ��
		laser_configuration();
		//��������ʼ�� TIM12
		buzzer_init(350, 90);
    //CAN�ӿڳ�ʼ��
		CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
		//��̨��ʼ��
//		GIMBAL_Setup();
		//���̳�ʼ��
		chassis_Setup();
		//��������ʼ��
//    shoot_init();
//		trigger_switch_init();
    shoot_setup();

		GM6020_PWM_configuration();//TIM5_CH1 H10 D   TIM5_CH2 H11 C ���ֿ���
    //24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
   
    remote_control_init(); 										//USART1 + DMA ң������ʼ��		
		BSPInit_CompleteBeep();
		buzzer_init(30000, 90);
		Referee_Task_Init();											//USART6 + DMA  ����ϵͳ��ȡ
		LX_IMU_UART8_Configuration(500000);				//UART8 �����ǳ�ʼ��
		ANO_DT_UART7_Configuration(500000);
		ANODT_DATA_Setup();
		
		LX_ANO_DT_Init();
		GPIOF_Exti0_GPIO_Init(); //��λ�����ⲿ�ж� ���������������λ���� F0 I2
		GPIOF_Exti1_GPIO_Init(); //��λ�����ⲿ�ж� ���ֵ�������λ���� F1	I1
		//��ʱ��6 ��ʼ��
    //TIM6_Init(60000, 90);
		//CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    //flash��ȡ��������У׼ֵ�Żض�Ӧ����
    //cali_param_init();
		//Calibration_Init();
}
