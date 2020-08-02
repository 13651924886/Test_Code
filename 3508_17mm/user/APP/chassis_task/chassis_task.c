/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      ��ɵ��̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 BDT****************************
  */
	
/************************* ͷ�ļ� ******************************/
	
#include "chassis_task.h"

#include "rc.h"

#include "arm_math.h"

#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "Remote_Control.h"
#include "INS_Task.h"
#include "delay.h"

#include "chassis_behaviour.h"
#include "referee_API.h"

/************************* �궨�� ******************************************************************/

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, ��û�в���ϵͳ���������µ����������� 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f		
/*********************** ȫ�ֱ������� ***************************************************************/
		

//�����˶����ݽṹ�嶨��
//static chassis_move_t chassis_move;
chassis_move_t chassis_move;


/*********************** ��̬�������� ***************************************************************/
		
//���̳�ʼ������Ҫ��pid��ʼ��
static void chassis_init(chassis_move_t *chassis_move_init);
//����״̬��ѡ��ͨ��ң�����Ŀ���
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//�������ݿ���ģʽ����
static void super_cap_mode_set(chassis_move_t *chassis_move);
//�������ݸ���
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//����״̬�ı����������ĸı�static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//�������ø���ң����������
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//����PID�����Լ��˶��ֽ�
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//���̹�������
static void chassis_power_control(chassis_move_t *chassis_power_control);

static void super_power_data_transmit(chassis_move_t *chassis_move);

/*********************** �������� *********************************************************************************/

void chassis_Setup(void)
{
    //���̳�ʼ��
    chassis_init(&chassis_move);
    //�жϵ��̵���Ƿ�����
//    while (toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE) || toe_is_error(DBUSTOE))
//    {
//        delay_ms(CHASSIS_CONTROL_TIME_MS);
//    }
}
//������
/************************ ������ ******************************************************/
void chassis_task(void)
{			
        
        //chassis_set_mode(&chassis_move);										//ң��������״̬
 
        //chassis_mode_change_control_transit(&chassis_move); //ң����״̬�л����ݱ���
 
        chassis_feedback_update(&chassis_move);//����chassis_move	->	motor_chassis[i].speed & accel �ٶȺͼ��ٶ�
																							 //		 chassis_move	->	vx & vy & wz									 �������᷽���ٶ� ������
																							 //		 chassis_move_update->chassis_yaw & pitch & roll ��������ŷ����   ������
	
        //chassis_set_contorl(&chassis_move); 	 //���̿��������ã�����ң�������������
	
        chassis_control_loop(&chassis_move);	 //���̿���PID����
	
				//super_power_data_transmit(&chassis_move);

	//�����Ұ�DetectTask������ȫ��ʡ�ԣ�Ϊ�˷��������ֲ����˲�����ң�����Ƿ���ߣ���һֱ�����������104�е�CAN_CMD_CHASSIS()
//        if (!(toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE)))
//        {
//            //��ң�������ߵ�ʱ��Ϊrelax״̬�����̵��ָ��Ϊ�㣬Ϊ�˱�֤һ������Ϊ�㣬�ʶ�����������give_current�ķ���
//            if (toe_is_error(DBUSTOE))
//            {
//                CAN_CMD_CHASSIS(0, 0, 0, 0);
//            }
//            else
//            {
//                CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
//                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
//            }
//        }  
				if (switch_is_mid(chassis_move.chassis_RC->rc.s[MODE_CHANNEL]))
				{
						CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
																		chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);		
				}
				//���ң������Ť�Ƿ��� -�� ���̲��ƶ�
				if (switch_is_down(chassis_move.chassis_RC->rc.s[MODE_CHANNEL]))
				{
						CAN_CMD_CHASSIS(0, 0,0,0);
																			
				}
				//���ң������Ť�Ƿ��� -�������涯
				else if (switch_is_up(chassis_move.chassis_RC->rc.s[MODE_CHANNEL]))
				{
					 CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
																		chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);		
				}
		
						

}

/*********************** �������ʼ��init ************************************************/
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //���̽ǶȻ�pidֵ
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    //����һ���˲�����
		const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //���̿���״̬Ϊֹͣ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;	//CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW
																													//CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW
																													//CHASSIS_VECTOR_NO_FOLLOW_YAW
/**** ң��������ָ���ȡ ****/																												//CHASSIS_VECTOR_RAW
    
    chassis_move_init->chassis_RC = get_remote_control_point();

/**** ����������ָ���ȡ ****/
    
		//��ȡ��������̬��ָ��&INS_Angle[3]   
		chassis_move_init->chassis_INS_angle = get_INS_angle_point();

/**** �������ָ���ȡ ****/
    
		//��ȡ��̨�������ָ�� pitch���yaw���
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

/**** ��ʼ��YAW��Pitch��PID���� ****/		
    
		//��ʼ��PID �˶�
    for (i = 0; i < 4; i++)
    {		//��ȡ���̵��m3508�ı�������CAN���߷��ص�����
				//һ��4��m3508��˽ṹ��������4����Ա
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        //��ʼ��4��������Ե��ٶȻ�PID������Ϊλ��ʽPID����motor_speed_pid[3]��Kp,Ki,Kd�������룬����޷�		
				PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //��ʼ���ǶȻ�PID������Ϊλ��ʽPID����chassis_yaw_pid��Kp,Ki,Kd�������룬����޷�										   
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
   
		//��һ���˲�����б����������
		//��ʼ����ſ���ʹ��һ�׵�ͨ�˲�
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //�޷�X�����Y������˶��ٶȡ�MAX and MIN��
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
		
		chassis_move_init->super_cap.super_power_mcu_msg_pointer = get_SuperPowerMcu_Msg_Point();
		
		chassis_move_init->super_cap.charge_cmd_state 		= Waiting;
		chassis_move_init->super_cap.charge_current_state = Waiting;
		chassis_move_init->super_cap.discharge_cmd_state	= Waiting;
		chassis_move_init->super_cap.discharge_current_state = Waiting;
		chassis_move_init->super_cap.voltage = 0.0f;
		

    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/************************************	1.chassis_Set_Mode ****************************************************/
//��һ��chassis_set_mode(chassis_move_t *chassis_move_mode)  																	[chassis_task.c 			static]

//�ڶ���	chassis_behaviour_mode_set(chassis_move_mode)																				[chassis_behaviour.c global]
//				����ң����ͨ��ֵ�����õ�����Ϊ״̬������chassis_behaviour_mode��ֵ
//				����6��ģʽ
//				 CHASSIS_ZERO_FORCE,                  //��������
//  			 CHASSIS_NO_MOVE,                     //���̱��ֲ���
//  			 CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //�����������̸�����̨
//  			 CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, //���̵��̽Ƕȿ��Ƶ��̣����ڵ���δ�������ǣ��ʶ��Ƕ��Ǽ�ȥ��̨�Ƕȶ��õ�������е�������������µ��̵�yaw��pitch��roll�Ƕ�
//  			 CHASSIS_NO_FOLLOW_YAW,               //���̲�����Ƕȣ��Ƕ��ǿ����ģ���ǰ�����������ٶȻ�
//  			 CHASSIS_OPEN                         //ң������ֵ���Ա���ֱ�ӷ��͵�can������
//
//				�ٸ��ݵ�����Ϊ״̬��chassis_behaviour_mode��ֵ��ֵ����״̬��chassis_move->chassis_mode
//				����3��״ֵ̬
// 				 CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,		//�����涯��̨
//   			 CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,		//�����涯����
//  			 CHASSIS_VECTOR_NO_FOLLOW_YAW,				//���̲��涯
//  			 CHASSIS_VECTOR_RAW,									//ԭʼ����

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	//����Ƿ������Ƿ�Ϊ��ָ��
	//����ǣ��򷵻أ���ֹ�������õ���ģʽ
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
		super_cap_mode_set(chassis_move_mode);
}
u8 move_count_flag = 0;
u8 count_flag = 0;
static void super_cap_mode_set(chassis_move_t *chassis_move)
{
    static uint16_t last_turn_keyboard = 0;
    static uint8_t mode_turn_flag = 0;
		if(chassis_move->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT )
		{
				chassis_move->super_cap.discharge_cmd_state = Discharging;
		}
		else
		{
				chassis_move->super_cap.discharge_cmd_state = Waiting;
		}
			
		if(	(chassis_move->chassis_RC->key.v & KEY_PRESSED_OFFSET_C) && !(last_turn_keyboard & KEY_PRESSED_OFFSET_C))
		{
				 if (mode_turn_flag == 0)
				 {
						mode_turn_flag = 1;								
				 }
		}
		
	  last_turn_keyboard = chassis_move->chassis_RC->key.v ;
		
	  if (mode_turn_flag)
	  {
				static u8 i = 0;
				if(chassis_move->super_cap.charge_cmd_state == Charging && i == 0 )
				{
						chassis_move->super_cap.charge_cmd_state = Waiting;
						i++;
				}
				if(chassis_move->super_cap.charge_cmd_state == Waiting && i == 0)	
				{
						chassis_move->super_cap.charge_cmd_state = Charging;
						i++;
				}
				i = 0;
				mode_turn_flag = 0;
		}
}
		



/**************************** 2.chassis_mode_change_control_transit ****************************/

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	//����Ƿ������Ƿ�Ϊ��ָ��
	//����ǣ��򷵻أ���ֹ�������õ���ģʽ
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //���������̽Ƕ�ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}


/********************************** 3.chassis_feedback_update ****************************************************/
//����chassis_move	->	motor_chassis[i].speed & accel �ٶȺͼ��ٶ�
//		chassis_move	->	vx & vy & wz									 �������᷽���ٶ�
//		chassis_move	->	chassis_yaw & pitch & roll ��������ŷ����
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
	/*********** ����chassis_move->motor_chassis[i].speed��accel ***********/
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ��������ʵ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *
																										( chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm);
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * 
																											CHASSIS_CONTROL_FREQUENCE;
    }

	/********** ���ݸ��µ�chassis_move->motor_chassis[i].speed������Vx��Vy��Wz ******************************/
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed 
																+ chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed
																)* MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed 
																+ chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed
																) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed
																- chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed
																) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

  /********** ���ݸ��µ�INS_Angle[]����̨��relative angleȥ���������̬ŷ����chassis_move_update->chassis_yaw & pitch & roll ******************************/

		//���������̬�Ƕ�, �����������������������ⲿ�ִ���
		//����yaw��̬�Ƕ� 	= INS_Angle[0] - ��̨yaw��relative_angle 	 ��rad��
		//����pitch��̬�Ƕ� = INS_Angle[1] - ��̨pitch��relative angle ��rad��
		//����roll��̬�Ƕ� 	= INS_Angle[2]														 ��rad��
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET)
																			 - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET)
																			 - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
				
		chassis_move_update->super_cap.discharge_current_state = chassis_move_update->super_cap.super_power_mcu_msg_pointer->msg1;
		chassis_move_update->super_cap.charge_current_state = chassis_move_update->super_cap.super_power_mcu_msg_pointer->msg2;
		chassis_move_update->super_cap.voltage = ((float)chassis_move_update->super_cap.super_power_mcu_msg_pointer->msg3) / 100.0f;
		
}

//ң���������ݴ���ɵ��̵�vx_set��vy_set
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //ң����ԭʼͨ��ֵ
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
		
//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = KEYBOARD_CHASSIS_SPEED_X;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = -KEYBOARD_CHASSIS_SPEED_X;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = KEYBOARD_CHASSIS_SPEED_Y;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = -KEYBOARD_CHASSIS_SPEED_Y;
    }

//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
		// In:vx_set_channel   Out:chassis_move->chassis_cmd_slow_set_vx
		// In:vy_set_channel   Out:chassis_move->chassis_cmd_slow_set_vy
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
		//���˻��������ٶ�Ϊ0
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}


/**************************** 4.chassis_set_contorl **********************************************************************************/
//���̿���������
//��һ��chassis_set_contorl(chassis_move_t *chassis_move_control)																[chassis_task.c 	global]

//�ڶ���	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control)		[chasssis_behaviour.c static]
//				���ݵ�����Ϊ״̬��chassis_behaviour_mode����Ӧ����vx_set��vy_set��angle_set	[local variables] 
//				
//�ڶ��� 	���ݵ��̿���״̬��chassis_move->chassis_mode������Ӧ��chassis_move->vx_set|vy_set|wz_set����
//

/****************  ��һ�� ���̿��������� ************************************************************************************/
//����ң�������������
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    //�����ٶ�
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
		
/******************�ڶ��� ���ݵ�����Ϊ״̬��chassis_behaviour_mode����Ӧ����vx_set��vy_set��angle_set[local variables] ***********************************/

//				 ����6��ģʽ
//				 CHASSIS_ZERO_FORCE,                  //��������
//  			 CHASSIS_NO_MOVE,                     //���̱��ֲ���
//  			 CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //�����������̸�����̨    ң����������->���̵�vx_set��vy_set , Ť��->angle_Set
//  			 CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, //���̵��̽Ƕȿ��Ƶ��̣����ڵ���δ�������ǣ��ʶ��Ƕ��Ǽ�ȥ��̨�Ƕȶ��õ�������е�������������µ��̵�yaw��pitch��roll�Ƕ�
//  			 CHASSIS_NO_FOLLOW_YAW,               //���̲�����Ƕȣ��Ƕ��ǿ����ģ���ǰ�����������ٶȻ�
//  			 CHASSIS_OPEN                         //ң������ֵ���Ա���ֱ�ӷ��͵�can������
		
   		chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

/******************�ڶ��� ���ݵ��̿���״̬��chassis_move->chassis_mode���յó�chassis_move->vx_set|vy_set|wz_set���� ***********************************/
//				 ����3��״ֵ̬
// 				 CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,		//�����涯��̨		��̨����ϵvx_set��vy_set�任����������ϵ��vx_set��vy_set
//   			 CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,		//�����涯����		
//  			 CHASSIS_VECTOR_NO_FOLLOW_YAW,				//���̲��涯
//  			 CHASSIS_VECTOR_RAW,									//ԭʼ����	
					/******************** ������̨YAWģʽ CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW ********************************************/
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;		//��̨����ϵ�任����������ϵ���˶�ѧ��ʽ
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;	//��̨����ϵ�任����������ϵ���˶�ѧ��ʽ
        //���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);

        //���ٶ����� = �ǶȻ�PID���

				chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid,
																									chassis_move_control->chassis_yaw_motor->relative_angle,
																									chassis_move_control->chassis_relative_angle_set);
//				chassis_move_control->wz_set = 10;
        //Vx������Vy�����޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, 
																											chassis_move_control->vx_min_speed,
																											chassis_move_control->vx_max_speed);
			
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, 
																											chassis_move_control->vy_min_speed,
																											chassis_move_control->vy_max_speed);
  
		}
		
		/******************** �������YAWģʽ CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW********************************************/		
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //����������̨
        //���õ��̿��ƵĽǶ�
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //������ת�Ľ��ٶ�
				
        chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
        //���õ����˶����ٶ�
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }

		/******************** ������YAWģʽ CHASSIS_VECTOR_NO_FOLLOW_YAW********************************************/		
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

        //����������̨
        //���ģʽ�£��Ƕ����õ�Ϊ ���ٶ�
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
		
		/******************** ��������ģʽ CHASSIS_VECTOR_RAW********************************************/		
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/****************** 5. chassis_control_loop *******************************************************************************************/

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
//    fp32 max_vector = 0.0f, vector_rate = 0.0f;
//    fp32 temp = 0.0f;
//    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
//    //�����˶��ֽ�
//    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
//                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

//    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
//    {
//        //��ֵ����ֵ
//        for (i = 0; i < 4; i++)
//        {
//            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
//        }
//        //raw����ֱ�ӷ���
//        return;
//    }

//    //�������ӿ�������ٶȣ�������������ٶ�
//    for (i = 0; i < 4; i++)
//    {
//        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
//        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
//        if (max_vector < temp)
//        {
//            max_vector = temp;
//        }
//    }

//    if (max_vector > MAX_WHEEL_SPEED)
//    {
//        vector_rate = MAX_WHEEL_SPEED / max_vector;
//        for (i = 0; i < 4; i++)
//        {
//            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
//        }
//    }

//    //����pid

		chassis_move_control_loop->motor_chassis[0].speed_set = 3.0;
		chassis_move_control_loop->motor_chassis[1].speed_set = -3.0;

    for (i = 0; i < 4; i++)
    {
        PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
		
		//���ʿ���
    //chassis_power_control(chassis_move_control_loop);
		
    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}

static void chassis_power_control(chassis_move_t *chassis_power_control)
{
		fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    uint8_t robot_id = get_robot_id();
//    if(toe_is_error(REFEREE_TOE))
//    {
//        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
//    }
    if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
        //���ʳ���80w �ͻ�������С��60j,��Ϊ��������С��60��ζ�Ź��ʳ���80w
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                //scale down WARNING_POWER_BUFF
                //��СWARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                //only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //scale down
            //��С
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //power > WARNING_POWER
            //���ʴ���WARNING_POWER
            if(chassis_power > WARNING_POWER)
            {
                fp32 power_scale;
                //power < 80w
                //����С��80w
                if(chassis_power < POWER_LIMIT)
                {
                    //scale down
                    //��С
                    power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                    
                }
                //power > 80w
                //���ʴ���80w
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //power < WARNING_POWER
            //����С��WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }

    
    total_current = 0.0f;
    //calculate the original motor current set
    //����ԭ����������趨
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }
    

    if(total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out*=current_scale;
        chassis_power_control->motor_speed_pid[1].out*=current_scale;
        chassis_power_control->motor_speed_pid[2].out*=current_scale;
        chassis_power_control->motor_speed_pid[3].out*=current_scale;
    }
}


static void super_power_data_transmit(chassis_move_t *chassis_move)
{		
		u8 msg1 = 0;
		u8 msg2 = 0;
		u8 msg3 = 0;
		u8 msg4 = 0;
		u8 msg5 = 0;
		u8 msg6 = 0;
		u8 msg7 = 0;
		u8 msg8 = 0;
		msg1 = chassis_move->super_cap.charge_cmd_state;
		msg2 = chassis_move->super_cap.discharge_cmd_state;
		SetSuperPowerMcuMsg(msg1,msg2,msg3,msg4,msg5,msg6,msg7,msg8);
	
}