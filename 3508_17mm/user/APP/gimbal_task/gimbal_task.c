/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
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

#include "Gimbal_Task.h"

#include "main.h"

#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "INS_Task.h"
#include "remote_control.h"
#include "shoot.h"
#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"
#include "delay.h"
#include "LX_IMU.h"

/************************* �궨�� ******************************************************************/

//�������ֵ���� 0��8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

/*********************** ȫ�ֱ������� ***************************************************************/

//��̨���������������
Gimbal_Control_t gimbal_control;

//���͵�can ָ��
//static int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0, Shoot_Can_Set_Current = 0;
int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0,Shoot_Can_Set_Current = 0;

		
/*********************** ��̬�������� ***************************************************************/

//��̨��ʼ��
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
//��̨pid����
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);
//��̨״̬����
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);
//��̨���ݸ���
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//��̨״̬�л��������ݣ������������״̬�л���������״̬����Ŀ��ֵ
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
//������̨��������ֵ����ԽǶ�
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
//������̨������
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//��̨����pid����
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);

static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);

//�������ǽǶȿ����£��Կ��Ƶ�Ŀ��ֵ�������Է��������ԽǶ�
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 *add);
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

static void calc_gimbal_cali(const Gimbal_Cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

Shoot_System_t shoot_t;
PidTypeDef shoot_pid; 

static void shoot_initiate(Shoot_System_t *shoot_t)
{
			static const fp32 Shoot_speed_pid[3] = {SHOOT_SPEED_PID_KP, SHOOT_SPEED_PID_KI, SHOOT_SPEED_PID_KD};
			shoot_t->shoot_rc_ctrl =  get_remote_control_point();
			shoot_t->shoot_motor.motor_measure = get_Shoot_Motor_Measure_Point();
			shoot_t->shoot_motor.speed_set = -10;
	    PID_Init(&shoot_pid, PID_POSITION, Shoot_speed_pid, SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);

}

void shoot_setup(void)
{
	shoot_initiate(&shoot_t);
}

void shoot_task(void)
{   
	  static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] 
										 + (shoot_t.shoot_motor.motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    shoot_t.shoot_motor.speed = speed_fliter_3;
	  PID_Calc(&shoot_pid, shoot_t.shoot_motor.speed, shoot_t.shoot_motor.speed_set);
	  shoot_t.shoot_motor.given_current = (int16_t)(shoot_pid.out);
    Shoot_Can_Set_Current = shoot_t.shoot_motor.given_current;
	
	  if (switch_is_mid(shoot_t.shoot_rc_ctrl->rc.s[0]))
		{
				CAN_CMD_GIMBAL(0, 0, Shoot_Can_Set_Current, 0 );	
		}
		//���ң������Ť�Ƿ��� ]
		if (switch_is_down(shoot_t.shoot_rc_ctrl->rc.s[0]))
		{
				CAN_CMD_GIMBAL(0, 0,0,0);
																	
		}
		//���ң������Ť�Ƿ��� ]
		else if (switch_is_up(shoot_t.shoot_rc_ctrl->rc.s[0]))
		{
     	  CAN_CMD_GIMBAL(0, 0, Shoot_Can_Set_Current, 0 );

		}
}
/*********************** �������� *********************************************************************************/

void GIMBAL_Setup(void)
{
    //��̨��ʼ��
    GIMBAL_Init(&gimbal_control);
	
    //�жϵ���Ƿ�����
//    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE) || toe_is_error(TriggerMotorTOE) )
//    {
//        delay_ms(GIMBAL_CONTROL_TIME);
//        GIMBAL_Feedback_Update(&gimbal_control);             //��̨���ݷ���
//    }
}

void GIMBAL_task(void)
{
        GIMBAL_Set_Mode(&gimbal_control);                    //������̨��Ϊ״̬�� -> ������̨���״̬�� -> �Ӷ������˶�ģʽ
        GIMBAL_Mode_Change_Control_Transit(&gimbal_control); //����ģʽ�л� �������ݹ���
        GIMBAL_Feedback_Update(&gimbal_control);             //����yaw��pitch��absolute_angle���ԽǶȡ�relative_angle��ԽǶȡ�motor_gyro���ٶ�
        GIMBAL_Set_Contorl(&gimbal_control);                 //ͨ��ң�������ݵõ�yaw��pitch��relative_angle_set/absolute_angle_set
        GIMBAL_Control_loop(&gimbal_control);                //��̨yaw��pitch����Ĵ���PID����
				shoot_control_loop();        //����������ѭ��
	
#if YAW_TURN		//YAW����Ƿ�װ
        Yaw_Can_Set_Current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN	//PITCH����Ƿ�װ
        Pitch_Can_Set_Current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

        //��̨��ң��������״̬��relax ״̬��canָ��Ϊ0����ʹ��current����Ϊ��ķ������Ǳ�֤ң��������һ��ʹ����ֹ̨ͣ
//        if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE) && toe_is_error(TriggerMotorTOE)))
//        {
//            if (toe_is_error(DBUSTOE))
//            {
//                CAN_CMD_GIMBAL(0, 0, 0, 0);
//            }
//            else
//            {
//                CAN_CMD_GIMBAL(Yaw_Can_Set_Current, Pitch_Can_Set_Current, Shoot_Can_Set_Current, 0);
//            }
//        }
				CAN_CMD_GIMBAL(Yaw_Can_Set_Current, Pitch_Can_Set_Current, Shoot_Can_Set_Current, 0 );

}


/**
  * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
  * @author         RM
  * @param[in]      yaw ��ֵ
  * @param[in]      pitch ��ֵ
  * @param[in]      yaw �����ԽǶ�
  * @param[in]      yaw ��С��ԽǶ�
  * @param[in]      pitch �����ԽǶ�
  * @param[in]      pitch ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}


/**
  * @brief          ��̨У׼���㣬��У׼��¼����� ��Сֵ ��������̨ ��ֵ�������С��е�Ƕ�
  * @author         RM
  * @param[in]      yaw ��ֵ ָ��
  * @param[in]      pitch ��ֵ ָ��
  * @param[in]      yaw �����ԽǶ� ָ��
  * @param[in]      yaw ��С��ԽǶ� ָ��
  * @param[in]      pitch �����ԽǶ� ָ��
  * @param[in]      pitch ��С��ԽǶ� ָ��
  * @retval         ����1 ����ɹ�У׼��ϣ� ����0 ����δУ׼��
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        //�������ʱ������ݣ���Ϊ��ʼ���ݣ����ж������Сֵ
        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;

        return 1;
    }
    else
    {
        return 0;
    }
}

//У׼���㣬������Ƕȣ���̨��ֵ
static void calc_gimbal_cali(const Gimbal_Cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / Motor_Ecd_to_Rad);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / Motor_Ecd_to_Rad);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ECD_Format(temp_max_ecd);
    ECD_Format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > Half_ecd_range)
    {
        temp_ecd -= ecd_range;
    }
    else if (temp_ecd < -Half_ecd_range)
    {
        temp_ecd += ecd_range;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ecd_range;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ECD_Format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / Motor_Ecd_to_Rad);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / Motor_Ecd_to_Rad);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ECD_Format(temp_max_ecd);
    ECD_Format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > Half_ecd_range)
    {
        temp_ecd -= ecd_range;
    }
    else if (temp_ecd < -Half_ecd_range)
    {
        temp_ecd += ecd_range;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ECD_Format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}
u8 chous7 = 0;
/************************************	GIMBAL_Init ****************************************************/
//��ʼ��pid ����ָ��
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
/*** YAW,PITCH��PID���� ****/
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		
/**** �������ָ���ȡ ****/
			//motor_yaw�ṹ����������Yaw��̨�����ʵʱ����
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();		
			//motor_pit�ṹ����������Pitch��̨�����ʵʱ����
		gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
		
/**** ����������ָ���ȡ ****/
			//INS_Angle[]���yaw,pitch,roll��ʵʱֵ
		gimbal_init->gimbal_INT_angle_point = get_LX_IMU_angle_point();
			//INS_gyro[]���������ٶ�ʵʱֵ
		gimbal_init->gimbal_INT_gyro_point = get_LX_IMU_gyro_point();
/**** ң��������ָ���ȡ ****/
		 //rc_ctrl�ṹ�壬���ң����ʵʱͨ��ֵ
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();	
		
/**** ��ʼ�����ģʽ ****/	
    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
		
/**** ��ʼ��YAW��Pitch��PID ���������PID���ֵ������������ ****/		
    //��ʼ��yaw������ԽǶȻ�PID
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, \
		YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT,\
		YAW_GYRO_ABSOLUTE_PID_KP,YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
		//��ʼ��yaw�����ԽǶȻ�PID
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid,\
		YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT,\
		YAW_ENCODE_RELATIVE_PID_KP,YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
		//��ʼ��yaw������ٶȻ�PID
	  PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid,\
		YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    
		//��ʼ��pitch������ԽǶȻ�PID
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid,\
		PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT,\
		PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
		//��ʼ��pitch�����ԽǶȻ�PID
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid,\
		PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT,\
		PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
		//��ʼ��pitch������ٶȻ�PID
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid,\
		PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);


    //�����̨PID�����������������ֵ��ȫ�����㣬��ֹ��������
    gimbal_total_pid_clear(gimbal_init);
		
/**** ����ʵ��װ����������ƫ��offset�������С��λ ****/
		//������̨relative angle��offset,����ʵ��װ����������
		gimbal_init->gimbal_yaw_motor.offset_ecd 	 = GIMBAL_YAW_RELATIVE_ANGLE_OFFSET;    //ȥ����У׼������������У׼��  2019-11-21 0:25:25 Stone
		gimbal_init->gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_RELATIVE_ANGLE_OFFSET;	//ȥ����У׼������������У׼��  2019-11-21 0:25:25 Stone
		
		//������̨relative angle���ֵ����Сֵ ����е��λ���ֵ
		gimbal_init->gimbal_yaw_motor.max_relative_angle = GIMBAL_YAW_MAX_RELATIVE_ANGLE;
		gimbal_init->gimbal_yaw_motor.min_relative_angle = GIMBAL_YAW_MIN_RELATIVE_ANGLE;
		gimbal_init->gimbal_pitch_motor.max_relative_angle = GIMBAL_PITCH_MAX_RELATIVE_ANGLE;
		gimbal_init->gimbal_pitch_motor.min_relative_angle = GIMBAL_PITCH_MIN_RELATIVE_ANGLE;
/**** ����yaw��pitch��absolute_angle���ԽǶȡ�relative_angle��ԽǶȡ�motor_gyro���ٶ� ****/		
    GIMBAL_Feedback_Update(gimbal_init);
		
		//����YAW��PITCH����Ϊ���ڵķ���
    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;

    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.relative_angle;
    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;


		
		chous7 = 1;
}
/************************************	1.GIMBAL_Set_Mode ****************************************************/
//��һ��GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)  																	[gimbal_task.c 			static]

//�ڶ���	gimbal_behaviour_mode_set(gimbal_set_mode)
//				����gimbal_behaviour��ֵȥ��gimbal_control->gimbal_yaw_motor.gimbal_motor_mode��ֵ	[gimbal_behaviour.c global]
//				����gimbal_behaviour��ֵȥ��gimbal_control->gimbal_pitch_motor.gimbal_motor_mode��ֵ
//				����3��״ֵ̬
// 				GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
//    		GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
//    		GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
//������		gimbal_behaviour_set(gimbal_mode_set)		����gimbal_control->gimbal_rc_ctrl->rc.s[ModeChannel]
//					���ڸ�ȫ�ֱ���gimbal_behaviour��ֵ																								[gimbal_behaviour.c static]
//					����6��״ֵ̬
//  				GIMBAL_ZERO_FORCE = 0, 	//��̨����
// 				  GIMBAL_INIT=1          	//��̨��ʼ��
//  				GIMBAL_CALI=2           //��̨У׼
// 					GIMBAL_ABSOLUTE_ANGLE=3 //��̨�����Ǿ��ԽǶȿ���
//  				GIMBAL_RELATIVE_ANGLE=4,//��̨����ֵ��ԽǶȿ���
//  				GIMBAL_MOTIONLESS=5     //��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��

static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
	//����Ƿ������Ƿ�Ϊ��ָ��
	//����ǣ��򷵻أ���ֹ����������̨ģʽ
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(gimbal_set_mode);
}


/**************************** 2.GIMBAL_Mode_Change_Control_Transit ��̨״̬�л����棬����״̬�л�����****************************/
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
	
//1.����Ƿ������Ƿ�Ϊ��ָ��
	
	//����ǣ��򷵻أ���ֹ����������̨ģʽ
    if (gimbal_mode_change == NULL)
    {
        return;
    }
		
//2.yaw���״̬���л���������
		
		//RAWģʽ ���������������ֵΪ��ǰ��������
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
		//���̸���ģʽ ������ԽǶ���������ֵΪ��ǰ���ԽǶ�
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
		//���̲�����ģʽ ������ԽǶ���������ֵΪ��ǰ��ԽǶ�
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
		//״̬����
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

//3.pitch���״̬���л���������
		//RAWģʽ ���������������ֵΪ��ǰ��������
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
		//���̸���ģʽ ������ԽǶ���������ֵΪ��ǰ���ԽǶ�
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
		//���̲�����ģʽ ������ԽǶ���������ֵΪ��ǰ��ԽǶ�
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }
		//״̬����
    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

/********************************** 3.GIMBAL_Feedback_Update ����yaw��pitch��absolute_angle���ԽǶȡ�relative_angle��ԽǶȡ�motor_gyro���ٶ� ****************************************************/
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
/*************************************** ��̨���ݸ��� ***********************************************************************************************/
		//���ܽ���:
		//����yaw��pitch��absolute_angle���ԽǶȡ�relative_angle��ԽǶȡ�motor_gyro���ٶ�
		/*****************************************************************************************************************************************************/
		//��������:
		//gimbal_feedback_update->gimbal_INT_angle_point == INS_Angle       ע:INS_Angle��INS_Angle[]������Ԫ�ص�ַ
		//INS_Angle[0]:yawʵʱ����ֵ,INS_Angle[1]:pitchʵʱ����ֵ,INS_Angle[2]:rollʵʱ����ֵ
		//gimbal_feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET = INS_Angle + 1 ��INS_Angle[1]�ĵ�ַ
		
		//gimbal_feedback_update->gimbal_INT_gyro_point == INS_gyro					ע:INS_gyro��INS_gyro[]������Ԫ�ص�ַ
		//INS_gyro[0]:x����ٶ�ʵʱֵ,INS_gyro[1]:y����ٶ�ʵʱֵ,INS_gyro[2]:z����ٶ�ʵʱֵ
		//gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET = INS_gyro + 1  ��INS_gyro[1]�ĵ�ַ
/*****************************************************************************************************************************************************/
//		gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = ((*(gimbal_feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET))/32768.0f) * 180.0f;
//    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
//                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd);
//    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET))/182.0f / 57.3f;

//    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = (((*(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET))/32768.0f)* 180.0f)/57.3f;
//    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
//                                                                                        gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);

//    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = (arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) *
//																												(((*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))/32768.0f)*2000.0f / 57.3f)
//                                                        - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * 
//																												(((*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET))/32768.0f)*2000.0f / 57.3f));
		//gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = ((*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))/32768.0f)*2000.0f / 57.3f;
		gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = (*(gimbal_feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET));
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd);
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = -(*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET));

    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = -(*(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET));
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);

    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) *
																												(*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * 
																												(*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}
//������ԽǶ�
//����GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range) ///�������ֵ��ֵHalf_ecd_range = 4096
    {
        relative_ecd -= ecd_range;		///�������ֵ���ֵecd_range = 8191
    }
    else if (relative_ecd < -Half_ecd_range)//-Half_ecd_range = -4096
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;//�������ֵת���ɽǶ�ֵ rad
}

/**************************** 4.GIMBAL_Set_Contorl **********************************************************************************/
//��̨����������
//��һ��GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)																	[gimbal_task.c 	global]

//�ڶ���	gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control)		[gimbal_behaviour.c global]
//				������̨��Ϊ״̬��gimbal_behaviour��ң������ͨ��ֵ������add_yaw_angle��add_pitch_angle
//				
//				������̨���״̬����ģʽ����yaw��pitch�Ƕ�����relative_angle_set/absolute_angle_set�޷�
//	

/****************  ��һ�� ��̨���������� ************************************************************************************/
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{		
    if (gimbal_set_control == NULL)
    {
        return;
    }
		fp32 add_yaw_angle = 0.0f;		//yaw��Ƕ���������
    fp32 add_pitch_angle = 0.0f;	//pitch��Ƕ���������

/******************�ڶ��� ������̨��Ϊ״̬��gimbal_behaviour����yaw��pitch����add_yaw_angle��add_pitch_angle ***********************************/
		
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);
		
/******************�ڶ��� ������̨���״̬����ģʽ����yaw��pitch�Ƕ�����relative_angle_set/absolute_angle_set�޷� *******************************************************/
	/****************** yaw����Ƕ�����relative_angle_set/absolute_angle_set�޷� ***************************************/
			if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
			{
					//rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
					gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
			}
			else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
			{
					//gyroģʽ�£������ǽǶȿ���
					GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
			}
			else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
			{
					//encondeģʽ�£��������Ƕȿ���
					GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, &add_yaw_angle);
			}
			
	/****************** pitch����Ƕ�����relative_angle_set/absolute_angle_set�޷� ***************************************/
			if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
			{
					//rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
					gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
			}
			else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
			{
					//gyroģʽ�£������ǽǶȿ���
					GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
			}
			else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
			{
					//encondeģʽ�£��������Ƕȿ���
					GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, &add_pitch_angle);
			}
}
//������ ����������
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle; 
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //���Ƕ� = ���ԽǶ�����absolute_angle_set - ���ԽǶȷ���absolute_angle_set
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //�����������е�Ƕȿ��Ʒ���
        if (add > 0.0f)
        {
            //�����һ��������ӽǶȣ�
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 *add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += *add;
    //�Ƿ񳬹���� ��Сֵ
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
		
}

/****************** 5. GIMBAL_Control_loop *******************************************************************************************/
/*************  ��̨����״̬ʹ�ò�ͬ����pid **********/
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }
	/************* yaw��̨�����ͬģʽ���ڲ�ͬ�Ŀ��ƺ��� *************/
    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw����
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //�����涯ģʽ ����Ϊabsolute_angle_set������Ϊabsolute_angle
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //���̲��涯ģʽ ����Ϊrelative_angle_set������Ϊrelative_angle
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }

	/************* pitch��̨�����ͬģʽ���ڲ�ͬ�Ŀ��ƺ��� *************/
    if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw����
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //�����涯ģʽ
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //���̲��涯ģʽ
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
}

/*************  GIMBAL_Control_loop()�и���ң����Switch��ֵ�����ò�ͬ�Ŀ��ƺ��� **********/
	//GIMBAL_MOTOR_GYROģʽʱ����
static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    
  /******************* yaw����̨����pid���� ***************************/
    //�ǶȻ��⻷	λ��ʽ
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid,
																										gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    //���ٶȻ��ڻ� λ��ʽ
		gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

	//GIMBAL_MOTOR_ENCONDEģʽʱ����
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

  /******************* yaw����̨����pid���� ***************************/
    //�ǶȻ��⻷	
		gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid,
																										gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    //���ٶȻ��ڻ�
		gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}




static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//pid��������
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
