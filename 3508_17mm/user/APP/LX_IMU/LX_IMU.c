#include "LX_IMU.h"
#include "uart8.h"

u8 send_buffer[50];	//�������ݻ���
fp32 Gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 yaw = 0;
u8 SHOCK_STA =0;
fp32 Angle[3] = {0.0f, 0.0f, 0.0f};      //ŷ���� ��λ rad Ϊ��DEBUG���㣬�Ұ�Static���η�ɾ����

_dt_st dt;
static u8 DT_RxBuffer[256],DT_data_cnt = 0;

//===================================================================
void LX_ANO_DT_Init(void)
{
	//========��ʱ����
	//
	dt.fun[0x0d].D_Addr = 0xff;
	dt.fun[0x0d].fre_ms = 100;   //�������͵�����100ms							
	dt.fun[0x0d].time_cnt_ms = 1;//���ó�ʼ��λ����λ1ms
	//
	dt.fun[0x40].D_Addr = 0xff;
	dt.fun[0x40].fre_ms = 100;   //�������͵�����100ms							
	dt.fun[0x40].time_cnt_ms = 0;//���ó�ʼ��λ����λ1ms
	//========�ⲿ����
	//
	dt.fun[0x30].D_Addr = 0xff;
	dt.fun[0x30].fre_ms = 0;     //0 ���ⲿ����
	dt.fun[0x30].time_cnt_ms = 0;//���ó�ʼ��λ����λ1ms	
	//
	dt.fun[0x33].D_Addr = 0xff;
	dt.fun[0x33].fre_ms = 0;     //0 ���ⲿ����
	dt.fun[0x33].time_cnt_ms = 0;//���ó�ʼ��λ����λ1ms
	//
	dt.fun[0x34].D_Addr = 0xff;
	dt.fun[0x34].fre_ms = 0;     //0 ���ⲿ����
	dt.fun[0x34].time_cnt_ms = 0;//���ó�ʼ��λ����λ1ms
	//
	dt.fun[0x41].D_Addr = 0xff;
	dt.fun[0x41].fre_ms = 0;     //0 ���ⲿ����
	dt.fun[0x41].time_cnt_ms = 0;//���ó�ʼ��λ����λ1ms
	//
	dt.fun[0xe0].D_Addr = 0xff;
	dt.fun[0xe0].fre_ms = 0;     //0 ���ⲿ����
	dt.fun[0xe0].time_cnt_ms = 0;//���ó�ʼ��λ����λ1ms
	//
	dt.fun[0xe2].D_Addr = 0xff;
	dt.fun[0xe2].fre_ms = 0;     //0 ���ⲿ����
	dt.fun[0xe2].time_cnt_ms = 0;//���ó�ʼ��λ����λ1ms
}

//���ݷ��ͽӿ�
static void ANO_DT_LX_Send_Data(u8 *dataToSend , u8 length)
{
	//
	Uart8_Send(dataToSend, length);
}

/**
  * @brief          ȡ������IMU�Ƕ�ֵָ��
  * @author         Stone
  * @param[in]      void
  * @retval         Angle[]������Ԫ�ص�ַ
  */
const fp32 *get_LX_IMU_angle_point(void)
{
    return Angle;
}

/**
  * @brief          ȡ������IMU���ٶ�ֵָ��
  * @author         Stone
  * @param[in]      void
  * @retval         Angle[]������Ԫ�ص�ַ
  */
const fp32 *get_LX_IMU_gyro_point(void)
{
    return Gyro;
}
//===================================================================
//���ݽ��ճ���
//===================================================================
void ANO_DT_LX_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0,_data_cnt = 0;
  static u8 rxstate = 0;
	
	if(rxstate==0&&data==0xAA)
	{
		rxstate=1;
		DT_RxBuffer[0]=data;
	}
	else if(rxstate==1 && (data == HW_TYPE || data == HW_ALL))
	{
		rxstate=2;
		DT_RxBuffer[1]=data;
	}
	else if(rxstate==2)
	{
		rxstate = 3;
		DT_RxBuffer[2]=data;
	}
	else if(rxstate==3&&data<250)
	{
		rxstate = 4;
		DT_RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(rxstate==4&&_data_len>0)
	{
		_data_len--;
		DT_RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			rxstate = 5;
	}
	else if(rxstate==5)
	{
		rxstate = 6;
		DT_RxBuffer[4+_data_cnt++]=data;
	}
	else if(rxstate==6)
	{
		rxstate = 0;
		DT_RxBuffer[4+_data_cnt]=data;
		DT_data_cnt = _data_cnt+5;
		//ano_dt_data_ok = 1;
		ANO_DT_LX_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt);
	}
	else
	{
		rxstate = 0;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���ANO_Data_Receive_Prepare�Զ�����
static void ANO_DT_LX_Data_Receive_Anl(u8 *data,u8 len)
{
	u8 check_sum1 = 0,check_sum2 = 0;
	if(*(data+3) != (len-6))	//�ж����ݳ����Ƿ���ȷ
		return;
  for(u8 i=0; i<len-2; i++)
	{
    check_sum1 += *(data+i);
		check_sum2 += check_sum1;
	}
	if((check_sum1 != *(data+len-2)) || (check_sum2 != *(data+len-1)))	//�ж�sumУ��
	{
    return;	
	}

  if(*(data)!=0xAA || (*(data+1)!=HW_TYPE && *(data+1)!=HW_ALL))
	return;
	//=============================================================================
//	if(*(data+2)==0X20) //PWM����
//	{
//		pwm_to_esc.pwm_m1 = *((u16 *)(data+4 ));
//		pwm_to_esc.pwm_m2 = *((u16 *)(data+6 ));
//		pwm_to_esc.pwm_m3 = *((u16 *)(data+8 ));
//		pwm_to_esc.pwm_m4 = *((u16 *)(data+10));
//		pwm_to_esc.pwm_m5 = *((u16 *)(data+12));
//		pwm_to_esc.pwm_m6 = *((u16 *)(data+14));
//		pwm_to_esc.pwm_m7 = *((u16 *)(data+16));
//		pwm_to_esc.pwm_m8 = *((u16 *)(data+18));
//	}
//	else if(*(data+2)==0X0f)//RGB
//	{
//		led.brightness[0] = *(data+4);
//		led.brightness[1] = *(data+5);
//		led.brightness[2] = *(data+6);
//		led.brightness[3] = *(data+7);		
//	}
//	else if(*(data+2)==0X06)
//	{
//		//
//		fc_sta.fc_mode_sta = *(data+4);
//		fc_sta.unlock_sta  = *(data+5);
//		fc_sta.cmd_fun.CID   = *(data+6);
//		fc_sta.cmd_fun.CMD_0 = *(data+7);
//		fc_sta.cmd_fun.CMD_1 = *(data+8);
//		//
//	}
	else if(*(data+2)==0x01)// ���ٶ�Acc|���ٶ�Gyro rad/s|�ȶ�״̬SHOCK_STA (����24g��Ӧs16�����ٶ�����2000��/���Ӧs16��-32768 - 32768��
	{
		Gyro[0] = *((s16 *)(data+10 ))* 0.00106f;		//X	(2000/32768) / 57.3 = 0.00106518597294938917975567190227
		Gyro[1] = *((s16 *)(data+12 ))* 0.00106f;		//Y	(2000/32768) / 57.3 = 0.00106518597294938917975567190227
		Gyro[2] = *((s16 *)(data+14 ))* 0.00106f;		//Z (2000/32768) / 57.3 = 0.00106518597294938917975567190227
		SHOCK_STA =  *((s16 *)(data+16 ));
	}
	else if(*(data+2)==0X03)//ŷ����
	{
		Angle[2] =  (*((s16 *)(data+4 )))/ 100.0f / 57.3f;		//Roll
		Angle[1] = (*((s16 *)(data+6 )))/100.0f / 57.3f;			//Pitch
		Angle[0] = 	(*((s16 *)(data+8 )))/100.0f / 57.3f;			//Yaw
		// = *((u16 *)(data+10));
	}
//	else if(*(data+2)==0XE0)			//����E0
//	{
//		switch(*(data+4))		//ID
//		{
//			case 0x01:
//			{
//			
//			}
//			break;
//			case 0x02:
//			{
//			
//			}
//			break;
//			case 0x10:
//			{
//			
//			}
//			break;
//			case 0x11:
//			{
//			
//			}
//			break;
//			default:
//				break;
//		}
//		//
//		dt.ck_send.ID = *(data+4);
//		dt.ck_send.SC = check_sum1;
//		dt.ck_send.AC = check_sum2;
//		CK_Back(SWJ_ADDR,&dt.ck_send);
//	}
//	else if(*(data+2)==0X00) //ck����
//	{
//		//
//		if((dt.ck_back.ID == *(data+4)) && (dt.ck_back.SC == *(data+5)) && (dt.ck_back.AC == *(data+6)))
//		{
//			dt.wait_ck = 0;//У��ɹ�
//		}
//		else
//		{
////			testCnt++;
//		}
//	}
//	else if(*(data+2)==0XE1)
//	{
//		//��ȡ����
//		u16 _par = *(data+4)+*(data+5)*256;
//		//
//		dt.par_data.par_id = _par;
//		dt.par_data.par_val = 0;
//		PAR_Back(0xff,&dt.par_data);
//	}
//	else if(*(data+2)==0xE2) 
//	{
////		//д�����
////		u16 _par = *(data+4)+*(data+5)*256;
////		u32 _val = (s32)(((*(data+6))) + ((*(data+7))<<8) + ((*(data+8))<<16) + ((*(data+9))<<24));
//		//
//		dt.ck_send.ID = *(data+4);
//		dt.ck_send.SC = check_sum1;
//		dt.ck_send.AC = check_sum2;
//		CK_Back(0xff,&dt.ck_send);
//		//��ֵ����
//		//Parameter_Set(_par,_val);
//		
//	}
}


////===================================================================
////���ݷ���ʵ�ֳ���
////===================================================================
//static void Add_Send_Data(u8 frame_num,u8 *_cnt,u8 send_buffer[])
//{
//	s16 temp_data;
//	s32 temp_data_32;
//	switch(frame_num)
//	{
//		case 0x00://CHECK����
//		{
//			send_buffer[(*_cnt)++]=dt.ck_send.ID;
//			send_buffer[(*_cnt)++]=dt.ck_send.SC;
//			send_buffer[(*_cnt)++]=dt.ck_send.AC;
//		}
//		break;
//		case 0x0d://�������
//		{
//			for(u8 i=0;i<4;i++)
//			{
//				send_buffer[(*_cnt)++]=fc_bat.byte_data[i];
//			}				
//		}
//		break;
//		case 0x30://GPS����
//		{
//			//
//			for(u8 i=0;i<23;i++)
//			{
//				send_buffer[(*_cnt)++]=ext_sens.fc_gps.byte[i];
//			}					
//		}
//		break;
//		case 0x33://ͨ���ٶȲ�������
//		{
//			//
//			for(u8 i=0;i<6;i++)
//			{
//				send_buffer[(*_cnt)++]=ext_sens.gen_vel.byte[i];
//			}					
//		}
//		break;
//		case 0x34://ͨ�þ����������
//		{
//			//
//			for(u8 i=0;i<7;i++)
//			{
//				send_buffer[(*_cnt)++]=ext_sens.gen_dis.byte[i];
//			}					
//		}
//		break;
//		case 0x40://ң������֡
//		{
//			for(u8 i=0;i<20;i++)
//			{
//				send_buffer[(*_cnt)++]=rc_in.rc_ch.byte_data[i];
//			}			
//		}
//		break;
//		case 0x41://ʵʱ��������֡
//		{
//			for(u8 i=0;i<14;i++)
//			{
//				send_buffer[(*_cnt)++]=rt_tar.byte_data[i];
//			}
//		}
//		break;	
//		case 0xe0://CMD����֡
//		{
//			send_buffer[(*_cnt)++]=dt.cmd_send.CID;
//			for(u8 i =0;i<10;i++)
//			{
//				send_buffer[(*_cnt)++]=dt.cmd_send.CMD[i];
//			}	
//		}
//		break;
//		case 0xe2://PARA����
//		{
//			temp_data = dt.par_data.par_id;
//			send_buffer[(*_cnt)++]=BYTE0(temp_data);
//			send_buffer[(*_cnt)++]=BYTE1(temp_data);
//			temp_data_32 = dt.par_data.par_val;
//			send_buffer[(*_cnt)++]=BYTE0(temp_data_32);
//			send_buffer[(*_cnt)++]=BYTE1(temp_data_32);			
//			send_buffer[(*_cnt)++]=BYTE2(temp_data_32);
//			send_buffer[(*_cnt)++]=BYTE3(temp_data_32);	
//		}
//		break;
//		default :break;
//	}
//}



////===================================================================

//static void Frame_Send(u8 frame_num,_dt_frame_st *dt_frame)
//{
//	u8 _cnt=0;
//	
//	send_buffer[_cnt++]=0xAA;
//	send_buffer[_cnt++]=dt_frame->D_Addr;
//	send_buffer[_cnt++]=frame_num;
//	send_buffer[_cnt++]=0;
//	//==
//	//add_send_data
//	Add_Send_Data(frame_num,&_cnt,send_buffer);
//	//==
//	send_buffer[3] = _cnt-4;
//	//==
//	u8 check_sum1 = 0, check_sum2 = 0;
//	for(u8 i=0;i<_cnt;i++)
//	{
//		check_sum1 += send_buffer[i];
//		check_sum2 += check_sum1;
//	}
//	send_buffer[_cnt++] = check_sum1;
//	send_buffer[_cnt++] = check_sum2;
//	//
//	if(dt.wait_ck !=0 && frame_num==0xe0)
//	{
//		dt.ck_back.ID = frame_num;
//		dt.ck_back.SC = check_sum1;
//		dt.ck_back.AC = check_sum2;
//	}
//	ANO_DT_LX_Send_Data(send_buffer, _cnt);
//}
////===================================================================
////
//static void Check_To_Send(u8 frame_num)
//{
//	//
//	if(dt.fun[frame_num].fre_ms)
//	{
//		//
//		if(dt.fun[frame_num].time_cnt_ms<dt.fun[frame_num].fre_ms)
//		{
//			dt.fun[frame_num].time_cnt_ms++;
//		}
//		else
//		{
//			dt.fun[frame_num].time_cnt_ms = 1;
//			dt.fun[frame_num].WTS = 1;//��ǵȴ�����
//		}		
//	}
//	else
//	{
//		//�ȴ��ⲿ����
//	}
//	//
//	if(dt.fun[frame_num].WTS)
//	{
//		dt.fun[frame_num].WTS = 0;
//		//ʵ�ʷ���
//		Frame_Send(frame_num,&dt.fun[frame_num]);
//	}
//}
////===================================================================

//CMD����
void CMD_Send(u8 dest_addr,_cmd_st *cmd)
{
	dt.fun[0xe0].D_Addr = dest_addr;
	dt.fun[0xe0].WTS=1;//���CMD�ȴ�����
	dt.wait_ck = 1;//��ǵȴ�У��
}
//CHECK����
void CK_Back(u8 dest_addr,_ck_st *ck)
{
	dt.fun[0x00].D_Addr = dest_addr;
	dt.fun[0x00].WTS=1;//���CMD�ȴ�����
}
//PARA����
void PAR_Back(u8 dest_addr,_par_st *par)
{
	dt.fun[0xe2].D_Addr = dest_addr;
	dt.fun[0xe2].WTS=1;//���CMD�ȴ�����
}

////��ָ��û���ͳɹ�����������·��ͣ����50ms��
//static u8 repeat_cnt;
//static inline void CK_Back_Check()
//{
//	static u8 time_dly;
//	if(dt.wait_ck == 1)
//	{
//		if(time_dly<50)//50ms
//		{
//			time_dly++;
//		}
//		else
//		{
//			time_dly = 0;
//			repeat_cnt++;
//			if(repeat_cnt<5)
//			{
//				dt.fun[0xe0].WTS = 1;//��ǵȴ����ͣ��ط�
//			}
//			else
//			{
//				repeat_cnt = 0;
//				dt.wait_ck = 0;
//			}
//		}
//	}
//	else
//	{
//		time_dly = 0;
//		repeat_cnt = 0;
//	}
//}

////1ms����һ�Σ�����ͨ�Ž�������
//void ANO_LX_Data_Exchange_Task(float dT_s)
//{
//	//=====���CMD�Ƿ񷵻���У��
//	CK_Back_Check();
//	//=====����Ƿ񴥷�����
//	Check_To_Send(0x30);
//	Check_To_Send(0x33);
//	Check_To_Send(0x34);
//	Check_To_Send(0x40);
//	Check_To_Send(0x41);
//	Check_To_Send(0xe0);
//	Check_To_Send(0xe2);
//	Check_To_Send(0x0d);

//	
//}