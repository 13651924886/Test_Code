/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      һ����ͨ������������豸�޴����̵�1Hz��˸,Ȼ���ȡ��̬��
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

#include "User_Task.h"
#include "main.h"


#include "led.h"

#include "Detect_Task.h"
#include "INS_Task.h"

#define user_is_error() toe_is_error(errorListLength)

//��̬�� ��λ��
//fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};

fp32 angle_degree2[3] = {0.0f, 0.0f, 0.0f};

//const volatile fp32 *angle;
const volatile short *angle2;

void UserTask_Setup(void)
{
   //��ȡ��̬��ָ��
   //angle = get_INS_angle_point();
	 //angle2 = get_WT_angle_point(); 		
}

void UserTask(void)
{
        //��̬�� ��rad ��� �ȣ����������̬�ǵĵ�λΪ�ȣ������ط�����̬�ǣ���λ��Ϊ����
//        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
//        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
//        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;
//				
				//angle_degree2[0] = (*(angle2 + INS_YAW_ADDRESS_OFFSET)) / 182.0f / 57.3f; 	//PITCH ̧ͷ���� ��ͷ��С
				//angle_degree2[1] = (*(angle2 + INS_PITCH_ADDRESS_OFFSET)) / 182.0f/ 57.3f; //
				//angle_degree2[2] = (*(angle2 + INS_ROLL_ADDRESS_OFFSET)) / 182.0f/ 57.3f; //YAW �����С

//        if (!user_is_error())
//        {
//            led_green_on();
//        }
//        vTaskDelay(500);
//        led_green_off();
//        vTaskDelay(500);
}
