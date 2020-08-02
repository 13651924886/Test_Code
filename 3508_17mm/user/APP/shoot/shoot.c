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

#include "Shoot.h"
#include "CAN_Receive.h"
#include "gimbal_behaviour.h"
#include "Detect_Task.h"
#include "pid.h"
#include "laser.h"
#include "fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "delay.h"




#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off() fric_off()         //�ر�����Ħ����

#define shoot_laser_on() laser_on()   //���⿪���궨��
#define shoot_laser_off() laser_off() //����رպ궨��

static const RC_ctrl_t *shoot_rc; //ң����ָ��

//static PidTypeDef trigger_motor_pid;         //���PID
PidTypeDef push_motor_pid; 
PidTypeDef shoot_motor_pid; 

//static Shoot_Motor_t trigger_motor;          //�������
//Shoot_Motor_t trigger_motor;          //�������

Shoot_System_t shoot_system;
//static shoot_mode_e shoot_mode = SHOOT_STOP; //���״̬��
//shoot_mode_e shoot_mode = SHOOT_STOP; //���״̬��

extern int16_t Push_Can_Set_Current,Shoot_Can_Set_Current;

//΢������IO
#define Butten_Trig_Pin GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10)

extern void getTriggerMotorMeasure(motor_measure_t *motor);

///////////////////// static ////////////////////////////
//���״̬������
static void Shoot_Set_Mode(void);
static void Shoot_Mode_Change_Control_Transit(void);
//������ݸ���
static void Shoot_Feedback_Update(void);
//���������
static void Shoot_Set_Control(void);
//PID����
static void Shoot_Calcualte(void);
//������ƣ����Ʋ�������Ƕȣ����һ�η���
static void shoot_bullet_control(void);
//�����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
static void shoot_done_control(void);
//���׼�����ƣ����ӵ��͵�΢�����ش���
static void shoot_ready_control(void);


/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
		//�����ٶ�PID
    static const fp32 Shoot_speed_pid[3] = {SHOOT_SPEED_PID_KP, SHOOT_SPEED_PID_KI, SHOOT_SPEED_PID_KD};
		//�����ٶ�PID
		static const fp32 Push_speed_pid[3] = {	PUSH_SPEED_PID_KP, PUSH_SPEED_PID_KI ,PUSH_SPEED_PID_KD};
    //ң����ָ��
		shoot_system.shoot_rc_ctrl = get_remote_control_point();
    shoot_rc = get_remote_control_point();
		//������λ����ָ��
		shoot_system.ShootTrigger = get_switch_point(ShootWheel);
		//������λ����ָ��
		shoot_system.PushTrigger = get_switch_point(PushWheel);
    //������ָ�루2006��
		shoot_system.shoot_motor.motor_measure = get_Shoot_Motor_Measure_Point();
		//ģʽ��ʼ��
		shoot_system.push_wheel_motor.mode = SHOOT_STOP;
		shoot_system.shoot_motor.mode = SHOOT_STOP;
		//��ʼ��ϵͳʱ��
		shoot_system.sys_time = SysTick_GetTick();
		//��ʼ�����ģʽ
		shoot_system.mouse.mode = 0;
		//��ʼ������ɹ���־
		shoot_system.shoot_done_flag = 0;
    //��ʼ��PID
    PID_Init(&shoot_motor_pid, PID_POSITION, Shoot_speed_pid, SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
		PID_Init(&push_motor_pid, PID_POSITION, Push_speed_pid, PUSH_SPEED_PID_MAX_OUT, PUSH_SPEED_PID_MAX_IOUT);
    //��������
    Shoot_Feedback_Update();
    ramp_init(&shoot_system.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);
    ramp_init(&shoot_system.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);

    shoot_system.shoot_motor.ecd_count = 0;
    shoot_system.shoot_motor.angle = shoot_system.shoot_motor.motor_measure->ecd * Motor_ECD_TO_ANGLE;
    shoot_system.shoot_motor.given_current = 0;
		shoot_system.shoot_motor.move_flag = 0;
		shoot_system.shoot_motor.shoot_count_flag = 0;
    shoot_system.shoot_motor.set_angle = shoot_system.shoot_motor.angle;
    shoot_system.shoot_motor.speed = 0.0f;
    shoot_system.shoot_motor.speed_set = 0.0f;
		
		shoot_system.push_wheel_motor.move_flag = 0;
//    triggshoot_system.pusher_motor.BulletShootCnt = 0;

}
/**
  * @brief          ���ѭ��
  * @author         RM
  * @param[in]      void
  * @retval         ����can����ֵ
  */

int16_t shoot_control_loop(void)
{
    Shoot_Set_Mode();        //����״̬��
		Shoot_Mode_Change_Control_Transit();
    Shoot_Feedback_Update(); //��������
		Shoot_Set_Control();
		Shoot_Calcualte();		
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
u8 sig = 0;
static void Shoot_Set_Mode(void)
{
		//���£��ر�ϵͳ���л���STOPģʽ
    if((switch_is_down(shoot_system.shoot_rc_ctrl->rc.s[Shoot_RC_Channel]))&& shoot_system.shoot_motor.move_flag == 0) //|| (shoot_system.shoot_rc_ctrl->key.v & SHOOT_OFF_KEYBOARD)
		{
        shoot_system.push_wheel_motor.mode = SHOOT_STOP;
				shoot_system.shoot_motor.mode			 = SHOOT_STOP;
    }	
	   //���ϻ���,����Ħ���֣����ֺͲ��治��
		if(	(switch_is_up(shoot_system.shoot_rc_ctrl->rc.s[Shoot_RC_Channel]) || switch_is_mid(shoot_system.shoot_rc_ctrl->rc.s[Shoot_RC_Channel]))
			&& shoot_system.shoot_motor.move_flag == 0)			 
		{
				shoot_system.push_wheel_motor.mode = SHOOT_WAITING;	// ģʽ�л����ȴ�Ħ��������
				shoot_system.shoot_motor.mode 		 = SHOOT_WAITING;	// ģʽ�л����ȴ�Ħ��������
		
				//����ϵͳ�����л����յ�ģʽ,�������ֿ�ʼ��ת��ת�ٺ�ΪPUSH_SPEED�����澲ֹ����
				if(	(shoot_system.fric1_ramp.out == shoot_system.fric1_ramp.max_value && shoot_system.fric2_ramp.out == shoot_system.fric2_ramp.max_value)&& shoot_system.shoot_motor.move_flag == 0	)
				{
						shoot_system.push_wheel_motor.mode =  AMMO_OUT;	// ģʽ�л�������
						shoot_system.shoot_motor.mode 		 = 	AMMO_OUT;	// ģʽ�л�������	
				}
						//ϵͳ����������⵽һ��������λ���ش�������������ֹͣ��ת,���濪ʼ��ת����
				if (shoot_system.PushTrigger->trigger_state == 1 && shoot_system.shoot_motor.move_flag == 0 )
				{
						shoot_system.push_wheel_motor.mode = SHOOT_INIT;
						shoot_system.shoot_motor.mode 		 = SHOOT_INIT;					
				}
				//����������λ���ش����󣬲�������ֹͣ��ת���ӵ��Ѿ�����������վ�����ʹ����깦�ܣ��ȴ�����ָ��
				if (shoot_system.ShootTrigger->trigger_state == 1 && shoot_system.shoot_motor.last_mode != SHOOTING )
				{		
						sig = 1;
						shoot_system.shoot_motor.mode 		 = SHOOT_READY;
						shoot_system.push_wheel_motor.mode = SHOOT_READY;
						shoot_system.shoot_motor.set_angle = shoot_system.shoot_motor.angle;
						shoot_system.shoot_motor.set_angle = rad_format(shoot_system.shoot_motor.set_angle + PI);
						//shoot_system.mouse.mode = 1;
						if (	 
								 shoot_system.shoot_motor.move_flag == 0 
								&& shoot_system.mouse.press_l 				== 1 
								&& shoot_system.mouse.last_press_l 		== 1
		//						shoot_system.mouse.mode						== 1 
							 )
						{

								shoot_system.shoot_motor.mode = SHOOTING;
								shoot_system.shoot_motor.move_flag = 1;			
						}
					}
			}

}

static void Shoot_Mode_Change_Control_Transit(void)
{
		shoot_system.shoot_motor.last_mode 			= shoot_system.shoot_motor.mode;
		shoot_system.push_wheel_motor.last_mode = shoot_system.push_wheel_motor.mode;
}
static void Shoot_Feedback_Update(void)
{
//////////////2006��� ����
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] 
										 + (shoot_system.shoot_motor.motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    shoot_system.shoot_motor.speed = -speed_fliter_3;

    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (shoot_system.shoot_motor.motor_measure->ecd - shoot_system.shoot_motor.motor_measure->last_ecd > Half_ecd_range)
    {
        shoot_system.shoot_motor.ecd_count--;
    }
    else if (shoot_system.shoot_motor.motor_measure->ecd - shoot_system.shoot_motor.motor_measure->last_ecd < -Half_ecd_range)
    {
        shoot_system.shoot_motor.ecd_count++;
    }

    if (shoot_system.shoot_motor.ecd_count == FULL_COUNT)
    {
        shoot_system.shoot_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_system.shoot_motor.ecd_count == -FULL_COUNT)
    {
        shoot_system.shoot_motor.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    shoot_system.shoot_motor.angle = -((shoot_system.shoot_motor.ecd_count * ecd_range + shoot_system.shoot_motor.motor_measure->ecd) * Motor_ECD_TO_ANGLE);

    //��갴��
    shoot_system.mouse.last_press_l = shoot_system.mouse.press_l;
    shoot_system.mouse.last_press_r = shoot_system.mouse.press_r;
    shoot_system.mouse.press_l = shoot_system.shoot_rc_ctrl->mouse.press_l;
    shoot_system.mouse.press_r = shoot_rc->mouse.press_r;
    //������ʱ
    if (shoot_system.mouse.press_l)
    {
        if (shoot_system.mouse.press_l_time < PRESS_LONG_TIME)
        {
            shoot_system.mouse.press_l_time++;
        }
    }
    else
    {
        shoot_system.mouse.press_l_time = 0;
    }

    if (shoot_system.mouse.press_r)
    {
        if (shoot_system.mouse.press_r_time < PRESS_LONG_TIME)
        {
            shoot_system.mouse.press_r_time++;
        }
    }
    else
    {
        shoot_system.mouse.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_system.shoot_motor.mode != SHOOT_STOP && switch_is_down(shoot_system.shoot_rc_ctrl->rc.s[Shoot_RC_Channel]))
    {

        if (shoot_system.mouse.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_system.mouse.rc_s_time++;
        }
    }
    else
    {
        shoot_system.mouse.rc_s_time = 0;
    }
}
u8 sisi1 = 0;

u8 sisi2 = 0;

static void Shoot_Set_Control(void)
{
	if ((shoot_system.push_wheel_motor.mode == SHOOT_STOP && shoot_system.shoot_motor.mode == SHOOT_STOP) || 
		(shoot_system.push_wheel_motor.mode == SHOOT_READY && shoot_system.shoot_motor.mode == SHOOT_READY)) 
//			&& shoot_system.shoot_motor.move_flag == 0)
	{
			shoot_system.push_wheel_motor.move_flag = 0;	
			shoot_system.shoot_motor.speed_set = 0.0f;		//BUGLIST1
		
			if (shoot_system.push_wheel_motor.mode == SHOOT_STOP && shoot_system.shoot_motor.mode == SHOOT_STOP)
			{
					shoot_system.fric1_ramp.out = Fric_OFF;
					shoot_system.fric2_ramp.out = Fric_OFF;
			}
	}
	if (shoot_system.push_wheel_motor.mode == AMMO_OUT && shoot_system.shoot_motor.mode == AMMO_OUT)	
	{
			shoot_system.push_wheel_motor.move_flag = 1;	
			shoot_system.shoot_motor.speed_set = 0.0f;
	}
	if (shoot_system.push_wheel_motor.mode == SHOOT_INIT && shoot_system.shoot_motor.mode == SHOOT_INIT)
	{
			shoot_system.push_wheel_motor.move_flag = 0;	
			shoot_system.shoot_motor.speed_set 			= SHOOT_SPEED;
	}
	
	if(	shoot_system.shoot_motor.mode == SHOOTING || shoot_system.shoot_motor.move_flag == 1 )
	{			
				shoot_system.shoot_motor.shoot_count_flag++;
				shoot_system.shoot_motor.speed_set 			= SHOOT_SPEED;
				if( shoot_system.shoot_motor.shoot_count_flag >= 1000)
				{
					shoot_system.shoot_motor.move_flag = 0;
					shoot_system.shoot_motor.shoot_count_flag = 0;
					shoot_system.push_wheel_motor.mode = SHOOT_INIT;
					shoot_system.shoot_motor.mode 		 = SHOOT_INIT;
					shoot_system.PushTrigger->trigger_state = 0;
					shoot_system.ShootTrigger->trigger_state = 0;
					shoot_system.mouse.mode = 0; //�������ʧ�ܣ��ȴ���һ���ӵ�׼������
				}
			
//			if ( shoot_system.shoot_motor.set_angle > shoot_system.shoot_motor.angle)
//			{
//				if (shoot_system.shoot_motor.set_angle - shoot_system.shoot_motor.angle > 0.05f)
//				{
//						//û����,���һֱ������ת�ٶ�
//						shoot_system.shoot_motor.speed_set = SHOOT_SPEED;					
//				}	
//				else
//				{
//						shoot_system.push_wheel_motor.mode = SHOOT_INIT;
//						shoot_system.shoot_motor.mode 		 = SHOOT_INIT;
//						shoot_system.shoot_motor.move_flag 			 = 0;
//						shoot_system.PushTrigger->trigger_state = 0;
//						shoot_system.ShootTrigger->trigger_state = 0;
//						//shoot_system.mouse.mode = 0; //�������ʧ�ܣ��ȴ���һ���ӵ�׼������
//				}							
//			}
//			else if(shoot_system.shoot_motor.set_angle < shoot_system.shoot_motor.angle)
//			{
//				if (shoot_system.shoot_motor.angle - shoot_system.shoot_motor.set_angle > 0.05f)
//				{
//						//û����,���һֱ������ת�ٶ�
//						sisi2 =1;
//						shoot_system.shoot_motor.speed_set = SHOOT_SPEED;					
//				}		
//				else
//				{
//						sisi2 = 1;
//						shoot_system.push_wheel_motor.mode = SHOOT_INIT;
//						shoot_system.shoot_motor.mode 		 = SHOOT_INIT;
//						shoot_system.shoot_motor.move_flag 			 = 0;
//						shoot_system.PushTrigger->trigger_state = 0;
//						shoot_system.ShootTrigger->trigger_state = 0;
//						//shoot_system.mouse.mode = 0; //�������ʧ�ܣ��ȴ���һ���ӵ�׼������
//				}							
//			}
//					
	}
			
}

static void Shoot_Calcualte(void)
{		
	  //Ħ����pwm
    static uint16_t fric_pwm1 = Fric_OFF;
    static uint16_t fric_pwm2 = Fric_OFF;
	
		if(shoot_system.push_wheel_motor.mode == SHOOT_STOP && shoot_system.shoot_motor.mode == SHOOT_STOP)
		{

        shoot_fric_off();
				Push_Wheel_Stop();
				Shoot_Can_Set_Current = 0;	
		}
		
    else
    {
				//shoot_laser_on();       //���⿪��
				
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_system.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
				//��һ��Ħ���ֿ������ٿ����ڶ���Ħ����
        if(shoot_system.fric1_ramp.out == shoot_system.fric1_ramp.max_value)
        {
            ramp_calc(&shoot_system.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        }
				//Ħ���ֻ�δ��ȫ������δ����ʱ������Ͳ���ֹͣת��
        if( shoot_system.fric2_ramp.out != shoot_system.fric2_ramp.max_value)
        {
            shoot_system.shoot_motor.speed_set = 0.0f;
						shoot_system.push_wheel_motor.move_flag = 0;
        }
				fric_pwm1 = (uint16_t)(shoot_system.fric1_ramp.out);
        fric_pwm2 = (uint16_t)(shoot_system.fric2_ramp.out);

        shoot_fric1_on(fric_pwm1);
        shoot_fric2_on(fric_pwm2);
//	//����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
//				static uint16_t up_time = 0;
//				if (shoot_system.mouse.press_r)
//				{
//						up_time = UP_ADD_TIME;
//				}

//				if (up_time > 0)
//				{
//						shoot_system.fric1_ramp.max_value = Fric_UP;
//						shoot_system.fric2_ramp.max_value = Fric_UP;
//						up_time--;
//				}
//				else
//				{
//						shoot_system.fric1_ramp.max_value = Fric_DOWN;
//						shoot_system.fric2_ramp.max_value = Fric_DOWN;
//				}
        //���㲦���ֵ��PID
        PID_Calc(&shoot_motor_pid, shoot_system.shoot_motor.speed, shoot_system.shoot_motor.speed_set);
				
        shoot_system.shoot_motor.given_current = (int16_t)(shoot_motor_pid.out);
		#if SHOOT_MOTOR_TURN				
						Shoot_Can_Set_Current = -shoot_system.shoot_motor.given_current;			
		#else
						Shoot_Can_Set_Current = shoot_system.shoot_motor.given_current;
						
		#endif
		if(shoot_system.push_wheel_motor.move_flag == 1)
		{
			#if PUSH_MOTOR_TURN
							Push_Wheel_CCW();
			#else
							Push_Wheel_CW();
			#endif
		}
		else 
		{
			Push_Wheel_Stop();
		}

    }
}	
				
/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
//static void shoot_bullet_control(void)
//{
//    //�ӵ�����ж�
////    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
////    {
////        trigger_motor.shoot_done = 1;
////        trigger_motor.shoot_done_time = 0;

////        shoot_mode = SHOOT_DONE;
////        trigger_motor.set_angle = trigger_motor.angle;
////    }

//    //ÿ�β��� 1/4PI�ĽǶ�
//    if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_BULLET)
//    {
//        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
//				trigger_motor.cmd_time = SysTick_GetTick();
//        trigger_motor.move_flag = 1;
//    }

//    //����Ƕ��ж�
//    if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//    {
//        //û����һֱ������ת�ٶ�
//        trigger_motor.speed_set = TRIGGER_SPEED;
//        trigger_motor.run_time = SysTick_GetTick();

//        //��ת�ж�
//        if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME &&
//					trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME &&
//					fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
//        {
//            trigger_motor.speed_set = -TRIGGER_SPEED;
//        }
//        else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
//        {
//            trigger_motor.cmd_time = SysTick_GetTick();
//        }
//    }
//    else
//    {
//        trigger_motor.move_flag = 0;
//    }
//}
///**
//  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
//  * @author         RM
//  * @param[in]      void
//  * @retval         void
//  */
//static void shoot_done_control(void)
//{
//    trigger_motor.speed_set = 0.0f;
//    //�������жϣ��ж�΢������һ��ʱ�����ӵ�
//    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
//    {
//        if (trigger_motor.shoot_done_time < SHOOT_DONE_KEY_OFF_TIME)
//        {
//            trigger_motor.shoot_done_time++;
//        }
//        else if (trigger_motor.shoot_done_time == SHOOT_DONE_KEY_OFF_TIME)
//        {
//            trigger_motor.BulletShootCnt++;
//            shoot_mode = SHOOT_READY;
//        }
//    }
//    else
//    {
//        shoot_mode = SHOOT_BULLET;
//    }
//}
///**
//  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
//  * @author         RM
//  * @param[in]      void
//  * @retval         void
//  */
//static void shoot_ready_control(void)
//{

//    if (trigger_motor.shoot_done)
//    {
//        trigger_motor.shoot_done = 0;
//    }

//    if (trigger_motor.key == SWITCH_TRIGGER_ON)
//    {
//        //�ж��ӵ�����΢�����ش�
//        trigger_motor.set_angle = trigger_motor.angle;
//        trigger_motor_pid.out = 0.0f;
//        trigger_motor_pid.Iout = 0.0f;

//        trigger_motor.speed_set = 0.0f;
//        trigger_motor.move_flag = 0;
//        trigger_motor.key_time = 0;
//    }
//    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time < KEY_OFF_JUGUE_TIME)
//    {
//        //�ж����ӵ�һ��ʱ��
//        trigger_motor.key_time++;
//    }
//    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time == KEY_OFF_JUGUE_TIME)
//    {
//        //΢������һ��ʱ��û���ӵ������벦����һ����ת 1/10PI�ĽǶ�
//        if (trigger_motor.move_flag == 0)
//        {
//            trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Ten);
//            trigger_motor.cmd_time = SysTick_GetTick();
//            trigger_motor.move_flag = 1;
//        }

//        if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//        {
//            //�Ƕȴﵽ�ж�
//            trigger_motor.speed_set = Ready_Trigger_Speed;
//            trigger_motor.run_time = SysTick_GetTick();
//            //��ת�ж�
//            if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
//            {
//                trigger_motor.speed_set = -Ready_Trigger_Speed;
//            }
//            else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
//            {
//                trigger_motor.cmd_time = SysTick_GetTick();
//            }
//        }
//        else
//        {
//            trigger_motor.move_flag = 0;
//        }
//    }
//}
