/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
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

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{



	CAN_CHASSIS_ALL_ID = 0x200,	//CAN���ͱ��ı�ʶ��
    CAN_3508_M1_ID = 0x201,			//CAN�������ı�ʶ��
    CAN_3508_M2_ID = 0x202,			//CAN�������ı�ʶ��
    CAN_3508_M3_ID = 0x203,			//CAN�������ı�ʶ��
    CAN_3508_M4_ID = 0x204,			//CAN�������ı�ʶ��

    CAN_YAW_MOTOR_ID = 0x205,		//CAN�������ı�ʶ��
    CAN_PIT_MOTOR_ID = 0x206,		//CAN�������ı�ʶ��
    CAN_SHOOT_ID = 0x207,//CAN�������ı�ʶ��
		CAN_PUSH_ID = 0x208,
		CAN_GIMBAL_ALL_ID = 0x1FF,	//CAN���ͱ��ı�ʶ��
	
		CAN_SetSuperPower_ID = 0x301,
		CAN_SuperPower_MCU_ID = 0x302,
		

} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t ecd;								//ת�ӻ�е�Ƕ�  0-8191��Ӧ0-360��
    int16_t speed_rpm;					//ת��ת�� ��λ��RPM
    int16_t given_current;			//���Ƶ��� -16384-0-16384��Ӧ-20-0-20A
    uint8_t temperate;					//����¶� ��λ����
    int16_t last_ecd;						//�ϴ�ת�ӻ�е�Ƕ�
} motor_measure_t;

typedef struct
{
		uint8_t msg1;
		uint8_t msg2;
		uint8_t	msg3;
		uint8_t	msg4;
		uint8_t msg5;
		uint8_t msg6;
		uint8_t msg7;
		uint8_t msg8;
		
} extended_mcu_msg_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//������̨�����������revΪ�����ֽ�
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//���͵��̵����������
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void SetSuperPowerMcuMsg(uint8_t msg1 ,uint8_t msg2, uint8_t msg3, uint8_t msg4,uint8_t msg5,uint8_t msg6,uint8_t msg7,uint8_t msg8);

//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//����Shoot���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Shoot_Motor_Measure_Point(void);
//����Push���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Push_Motor_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

extern const extended_mcu_msg_t *get_SuperPowerMcu_Msg_Point(void);


#endif
