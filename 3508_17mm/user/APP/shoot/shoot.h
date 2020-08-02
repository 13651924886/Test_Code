/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܣ���������ĳ�ʼ�����Լ�ѭ����������̨�����е��ã��ʶ����ļ�
  *             ����freeRTOS��������ֹر�״̬��׼��״̬�����״̬���Լ����״̬
  *             �ر�״̬�ǹر�Ħ�����Լ����⣬׼��״̬�ǽ��ӵ�����΢�Ϳ��ش������״
  *             ̬�ǽ��ӵ�������ж�΢�Ϳ���ֵ���������״̬�����״̬ͨ���ж�һ��ʱ��
  *             ΢�Ϳ������ӵ���Ϊ�Ѿ����ӵ������
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

#ifndef SHOOT_H
#define SHOOT_H
#include "main.h"

#include "push_motor.h"
#include "CAN_Receive.h"
#include "remote_control.h"
#include "user_lib.h"
#include "Gimbal_Task.h"
#include "trigger_switch.h"


#define SHOOT_MOTOR_TURN 1	//0������װ  1����װ
#define PUSH_MOTOR_TURN 0
//������俪��ͨ������
#define Shoot_RC_Channel    1
//��̨ģʽʹ�õĿ���ͨ��
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  1 //1ms��������

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME 10
//��곤���ж�
#define PRESS_LONG_TIME 400
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME 80
//�����������ֵ��Χ
#define Half_ecd_range 4096
#define ecd_range 8191
//2006���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f  // ����2006 ������Ϊ36 ��ô��ϵ�� =��10��/8191�� *��PI/180��
#define FULL_COUNT 18
//�����ٶ�
#define SHOOT_SPEED 5.0f
#define Ready_Trigger_Speed 6.0f

//6020���RPM �仯�� ��ת�ٶȵı���
#define PUSH_Motor_RMP_TO_SPEED -0.1047f // ������Ϊ1 ���1*2*PI/60 
//�����ٶ�
#define PUSH_SPEED -10.0f //��λrad/s


#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0
#define SWITCH_TRIGGER_OFF 1

//����ʱ�� �Լ���תʱ��
#define BLOCK_TIME 700
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_TWO 1.5707963267948966192313216916398f
#define PI_Four 0.78539816339744830961566084581988f
#define PI_Ten 0.314f

//�����ֵ��PID
#define SHOOT_SPEED_PID_KP 3000.0f
#define SHOOT_SPEED_PID_KI 0.3f
#define SHOOT_SPEED_PID_KD 0.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define SHOOT_SPEED_PID_MAX_OUT 5000.0f
#define SHOOT_SPEED_PID_MAX_IOUT 2500.0f

#define PUSH_SPEED_PID_KP 1000.0f
#define PUSH_SPEED_PID_KI 0.0f
#define PUSH_SPEED_PID_KD 0.0f

#define PUSH_SPEED_PID_MAX_OUT 5000.0f
#define PUSH_SPEED_PID_MAX_IOUT 2500.0f

/* TODO1 ���ƽṹ��*/
typedef enum
{
		SHOOT_STOP = 0,
		SHOOT_WAITING,
    AMMO_OUT,
		SHOOT_INIT,
    SHOOT_READY,
		SHOOTING,
		SHOOT_JAMMED,
    SHOOT_DONE,
} shoot_system_motor_mode_e;

typedef struct
{
	const motor_measure_t *motor_measure;
	shoot_system_motor_mode_e mode;
	shoot_system_motor_mode_e last_mode;
	bool_t move_flag;
	fp32 shoot_count_flag;
	fp32 speed;
  fp32 speed_set;
  fp32 angle;
  fp32 set_angle;
  int16_t given_current;
  int8_t ecd_count;
	
} Shoot_System_Shoot_Motor_t;

typedef struct
{
	shoot_system_motor_mode_e mode;
	shoot_system_motor_mode_e last_mode;
	bool_t move_flag;
  //fp32 angle;
  //fp32 set_angle;
	
} Shoot_System_Push_Wheel_t;

typedef struct
{
	bool_t mode;	// 0��1 �Ƿ��������ڷ���
		
	bool_t press_l;
  bool_t press_r;
  bool_t last_press_l;
  bool_t last_press_r;
  uint16_t press_l_time;
  uint16_t press_r_time;
  uint16_t rc_s_time;
	
}	Mouse_t;

typedef struct
{
		const RC_ctrl_t *shoot_rc_ctrl;	//ң����ָ��
	
    ramp_function_source_t fric1_ramp;//Ħ����б��
    ramp_function_source_t fric2_ramp;
	
		Shoot_System_Shoot_Motor_t shoot_motor;			//����
		Shoot_System_Push_Wheel_t push_wheel_motor;	//����

		Mouse_t mouse;	//���
		
		u32 sys_time;	//ϵͳʱ��T
	
		trigger_switch_t *PushTrigger;
		trigger_switch_t *ShootTrigger;
	
		bool_t shoot_done_flag;
	
//    bool_t move_flag;
//    uint32_t cmd_time;
//    uint32_t run_time;
//    bool_t key;
//    uint16_t key_time;
//    bool_t shoot_done;
//    uint8_t shoot_done_time;
//    int16_t BulletShootCnt;
//    int16_t last_butter_count;
} Shoot_System_t;


extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
