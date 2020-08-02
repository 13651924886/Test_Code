/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2019-11-16     	Stone              1. ���
  *
  @verbatim
  ==============================================================================
	�������̽��ͣ�
	
		�����CAN1���߷��ͷ������� ---->CAN1_RX0_IRQHandler() ----> CAN_hook() ----> get_motor_measure(ptr, rx_message)
																								 �� get_gimbal_motor_measuer(ptr, rx_message)
																								 
		�����CAN2���߷��ͷ������� ---->CAN2_RX0_IRQHandler() ----> CAN_hook() ----> get_motor_measure(ptr, rx_message)
																								 �� get_gimbal_motor_measuer(ptr, rx_message)
						������Ϣ�ֱ�����motor_yaw, motor_pit, motor_trigger, motor_chassis[4]�ṹ�������
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

/************************* ͷ�ļ� ******************************/

#include "CAN_Receive.h"

#include "stm32f4xx.h"
#include "rng.h"

#include "Detect_Task.h"


/*********************** �궨�庯�� ****************************/

//���̵�����ݶ�ȡ
//��CAN_hook()��ʹ��
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//��̨������ݶ�ȡ
//CAN_hook()��ʹ��
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
		
#define get_super_power_mcu_msg(ptr, rx_message)																							\
		{																																													\
				(ptr)->msg1 = (rx_message)->Data[0]; 					\
				(ptr)->msg2 = (rx_message)->Data[1];					\
				(ptr)->msg3 = (rx_message)->Data[2];					\
				(ptr)->msg4 =	(rx_message)->Data[3];					\
				(ptr)->msg5 =	(rx_message)->Data[4];					\
				(ptr)->msg6 =	(rx_message)->Data[5];					\
				(ptr)->msg7 =	(rx_message)->Data[6];					\
				(ptr)->msg8 =	(rx_message)->Data[7];					\
		}	
/************************ �������� ******************************/
		
//���Ľ�������
//CAN1_RX0_IRQHandler()��ʹ��
//CAN2_RX0_IRQHandler()��ʹ��
static void CAN_hook(CanRxMsg *rx_message);
		
		
		
/*********************** ȫ�ֱ������� ***************************/
//�����������
motor_measure_t motor_yaw, motor_pit, motor_push, motor_shoot , motor_chassis[4]; //����Debug�鿴���� �Ѿ�̬ȥ��
		
extended_mcu_msg_t	super_power_mcu_msg;

static CanTxMsg GIMBAL_TxMessage;
		
		
		
		
/************************ �������� ******************************/
//can1�����жϷ�����
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN_hook(&rx1_message);
    }
}

//can2�����жϷ�����
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN_hook(&rx2_message);
    }
}

//������̨�����������revΪ�����ֽ�
//Gimbal_Task����
void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = (rev >> 8);
    GIMBAL_TxMessage.Data[7] = rev;
    CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );


}

void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}
//CAN ���� 0x700��ID�����ݣ�������M3508�����������IDģʽ
//��������û��ʹ��
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN2, &TxMessage);
}

//���͵��̵����������
//��chassis_task��ʹ��
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//����Shoot���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Shoot_Motor_Measure_Point(void)
{
    return &motor_shoot;
}
//����Push���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Push_Motor_Measure_Point(void)
{
    return &motor_push;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];//i & 0000 0011
}

const extended_mcu_msg_t *get_SuperPowerMcu_Msg_Point(void)
{
    return &super_power_mcu_msg;//i & 0000 0011
}

void SetSuperPowerMcuMsg(uint8_t msg1 ,uint8_t msg2, uint8_t msg3, uint8_t msg4,uint8_t msg5,uint8_t msg6,uint8_t msg7,uint8_t msg8)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    can_id = CAN_SetSuperPower_ID; 
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = msg1;
    tx_message.Data[1] = msg2;
    tx_message.Data[2] = msg3;
    tx_message.Data[3] = msg4;
    tx_message.Data[4] = msg5;
    tx_message.Data[5] = msg6;
    tx_message.Data[6] = msg7;
    tx_message.Data[7] = msg8;
    
    CAN_Transmit(CAN1,&tx_message);
}


//CAN���ߵ���������Ľ������������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
//��ʱȥ��DetectTask����
static void CAN_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
        //���������ݺ꺯��
        get_gimbal_motor_measuer(&motor_yaw, rx_message);
        //��¼ʱ��
        //DetectHook(YawGimbalMotorTOE);  //��ʱȥ��DetectTask����
        break;
    }
    case CAN_PIT_MOTOR_ID:
    {
        //���������ݺ꺯��
        get_gimbal_motor_measuer(&motor_pit, rx_message);
        //DetectHook(PitchGimbalMotorTOE);
        break;
    }
    case CAN_SHOOT_ID:
    {
        //���������ݺ꺯��
        get_motor_measure(&motor_shoot, rx_message);
        //��¼ʱ��
        //DetectHook(TriggerMotorTOE);  //��ʱȥ��DetectTask����
        break;
    }
		case CAN_PUSH_ID:
		{
			  get_gimbal_motor_measuer(&motor_push, rx_message);
				break;
		}
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //������ID��
        i = rx_message->StdId - CAN_3508_M1_ID;
        //���������ݺ꺯��
        get_motor_measure(&motor_chassis[i], rx_message);
        //��¼ʱ��
        //DetectHook(ChassisMotor1TOE + i); //��ʱȥ��DetectTask����
        break;
    }
		case CAN_SuperPower_MCU_ID:
		{
				get_super_power_mcu_msg(&super_power_mcu_msg, rx_message);		
				break;
		}

    default:
    {
        break;
    }
    }
}
