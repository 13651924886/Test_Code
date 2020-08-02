/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       INSTask.c/h
  * @brief      ��Ҫ����������mpu6500��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��mpu6500��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ�䣬�ṩע�Ͷ�Ӧ�ĺ궨�壬�ر�DMA��
  *             DR���ⲿ�жϵķ�ʽ.
  * @note       SPI �������ǳ�ʼ����ʱ����Ҫ����2MHz��֮���ȡ���������20MHz
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

/************************* ͷ�ļ� ******************************/

#include "INS_Task.h"

#include "stm32f4xx.h"

#include "buzzer.h"
#include "timer.h"
#include "spi.h"
#include "exit_init.h"
#include "IST8310driver.h"
#include "mpu6500driver.h"
#include "mpu6500reg.h"
#include "mpu6500driver_middleware.h"

#include "AHRS.h"

#include "calibrate_Task.h"
#include "pid.h"



/************************* �궨�� ******************************/

#define IMUWarnBuzzerOn() buzzer_on(95, 10000) //����������У׼������

#define IMUWarnBuzzerOFF() buzzer_off() //����������У׼�������ر�

#define MPU6500_TEMPERATURE_PWM_INIT() TIM3_Init(MPU6500_TEMP_PWM_MAX, 1) //�������¶ȿ���PWM��ʼ��
#define IMUTempPWM(pwm) TIM_SetCompare2(TIM3, (pwm))                      //pwm����
#define INS_GET_CONTROL_TEMPERATURE() get_control_temperate()             //��ȡ�����¶ȵ�Ŀ��ֵ

#if defined(MPU6500_USE_DATA_READY_EXIT)

#define MPU6500_DATA_READY_EXIT_INIT() GPIOB_Exti8_GPIO_Init() //��ʼ��mpu6500�� �ⲿ�ж� ʹ��PB8 �ⲿ�ж��� 8

#define MPU6500_DATA_READY_EXIT_IRQHandler EXTI9_5_IRQHandler //�궨���ⲿ�жϺ�����ʹ����line8�ⲿ�ж�

#define MPU6500_DATA_READY_EXIT_Line EXTI_Line8 //�궨���ⲿ�ж���
#endif

#if defined(MPU6500_USE_SPI) && defined(MPU6500_USE_SPI_DMA)

//�궨���ʼ��SPI��DMA��ͬʱ����SPIΪ8λ��4��Ƶ
#define MPU6500_SPI_DMA_Init(txbuf, rxbuf)                                 \
    {                                                                      \
        SPI5_DMA_Init((uint32_t)txbuf, (uint32_t)rxbuf, DMA_RX_NUM);       \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Rx, ENABLE);                   \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Tx, ENABLE);                   \
        SPI5SetSpeedAndDataSize(SPI_BaudRatePrescaler_8, SPI_DataSize_8b); \
    }

#define MPU6500_SPI_DMA_Enable() SPI5_DMA_Enable(DMA_RX_NUM) // ��ʼһ��SPI��DMA����
//�궨��SPI��DMA�����жϺ����Լ������жϱ�־λ
#define MPU6500_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define MPU6500_DMA_Stream DMA2_Stream5
#define MPU6500_DMA_FLAG DMA_FLAG_TCIF5
#elif defined(MPU6500_USE_SPI_DMA)
#error "the communication of mpu6500 is not SPI, can't use the DMA"
#endif


//DMA��SPI ���͵�buf����INT_STATUS��ʼ������ȡ DMA_RX_NUM��С��ַ��ֵ
#if defined(MPU6500_USE_SPI_DMA)
static const uint8_t mpu6500_spi_DMA_txbuf[DMA_RX_NUM] =
    {
        MPU_INT_STATUS | MPU_SPI_READ_MSB};
#endif


#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \

																			
																				

																		
/*********************** ȫ�ֱ������� ***************************/
								
//���������ǣ����ٶȼƣ����������ݵ����Զȣ���Ư
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310);
static void IMU_temp_Control(fp32 temp);

uint8_t mpu6500_spi_rxbuf[DMA_RX_NUM]; //������յ�ԭʼ����
//static mpu6500_real_data_t mpu6500_real_data; //ת���ɹ��ʵ�λ��MPU6500����
mpu6500_real_data_t mpu6500_real_data; //ת���ɹ��ʵ�λ��MPU6500����
																				
static fp32 Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //������У׼���Զ�

//static fp32 gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};
fp32 gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};

//static fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //��������Ư
fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //��������Ư
static fp32 Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //���ٶ�У׼���Զ�
static fp32 Accel_Offset[3] = {0.0f, 0.0f, 0.0f};            //���ٶ���Ư
static ist8310_real_data_t ist8310_real_data;                //ת���ɹ��ʵ�λ��IST8310����
static fp32 Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
                                      {0.0f, 1.0f, 0.0f},
                                      {0.0f, 0.0f, 1.0f}}; //������У׼���Զ�
//static fp32 Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //��������Ư
fp32 Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //��������Ư

static const float TimingTime = INS_DELTA_TICK * 0.001f;   //�������е�ʱ�� ��λ s

fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

fp32 INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //ŷ���� ��λ rad Ϊ��DEBUG���㣬�Ұ�Static���η�ɾ����
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //��Ԫ��

static const fp32 imuTempPID[3] = {MPU6500_TEMPERATURE_PID_KP, MPU6500_TEMPERATURE_PID_KI, MPU6500_TEMPERATURE_PID_KD};
static PidTypeDef imuTempPid;

static uint8_t first_temperate = 0;

//KZ_Filter()����
fp32 k = 0;
fp32 sum = 0;
fp32 Value = 0.;
fp32 NewValue;
fp32 new_value = 0.000000000001;
fp32 kzkz = 0;
fp32 sdfor = 1.73e-05;//��Ưϵ������flashд�룬ֵԽ����Ư����Խ�ã�4e-05ʱ����������Ư������Խ��Խ���׵��²����ӳ�
//ƽ���ȴ���
fp32 filter_sum = 0;
#define FILTER_N 12
fp32 coe[FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // ��Ȩϵ����
fp32 sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // ��Ȩϵ����
fp32 filter_buf[FILTER_N + 1];

static fp32 KZ_Filter(void)
{
	kzkz = NewValue - INS_Angle[0];
  NewValue = INS_Angle[0];
  if((kzkz > sdfor) || (kzkz < ( - sdfor)))
  { new_value = Value;
    Value = NewValue - sum;
  }
  else
      sum += kzkz;
  
  filter_buf[FILTER_N] = Value;
  for(uint32_t i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // �����������ƣ���λ�Ե�
    filter_sum += filter_buf[i] * coe[i];
  }
  filter_sum /= sum_coe;
  k = (filter_sum /3.14)*180;//�۲�ֵ��360�ȵ�λ
  return filter_sum;
}

/************************ �������� ******************************/

/**
  * @brief          IMU��ֵ����
  * @author         Stone
  * @param[in]      void
  * @retval         void
  */
void IMU_Update(void)
{
			//����ȡ����mpu6500ԭʼ���ݴ���ɹ��ʵ�λ������
        mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET), &mpu6500_real_data);
			//����ȡ����ist8310ԭʼ���ݴ���ɹ��ʵ�λ������
        ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET), &ist8310_real_data);
        //��ȥ��Ư�Լ���ת����ϵ
        IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);


        //���ٶȼƵ�ͨ�˲�
        static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
        static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
        static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
        static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


        //�ж��Ƿ��һ�ν��룬�����һ�����ʼ����Ԫ����֮�������Ԫ������Ƕȵ�λrad
        static uint8_t updata_count = 0;

        if( mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT)//#define MPU_DATA_READY_BIT 0 //����������׼��
        {
						//�ж��Ƿ�Ϊ��һ�ν���
						//����ǣ���ô���룬��һ�γ�ʼ���¿غ���Ԫ��
            if (updata_count == 0)
            {		
						/*IMU�¿ز��֣���ʱ����*/
                //MPU6500_TEMPERATURE_PWM_INIT();
                //PID_Init(&imuTempPid, PID_DELTA, imuTempPID, MPU6500_TEMPERATURE_PID_MAX_OUT, MPU6500_TEMPERATURE_PID_MAX_IOUT);
						/*IMU�¿ز��֣���ʱ����*/
							
                //���ݼ��ٶȵ�����INS_accel[]�������Ƶ�����INS_mag[]������Ԫ��INS_quat[]��ʼ��
                AHRS_init(INS_quat, INS_accel, INS_mag);
								//������Ԫ��INS_quat[]��С�����Ӧ��ŷ����yaw(INS_Angle[0])��pitch(INS_Angle[1])��roll(INS_Angle[2])
                get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

                accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
                accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
                accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
                updata_count++;
            }
            else
            {
                //���ٶȼƵ�ͨ�˲�
                accel_fliter_1[0] = accel_fliter_2[0];
                accel_fliter_2[0] = accel_fliter_3[0];

                accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

                accel_fliter_1[1] = accel_fliter_2[1];
                accel_fliter_2[1] = accel_fliter_3[1];

                accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

                accel_fliter_1[2] = accel_fliter_2[2];
                accel_fliter_2[2] = accel_fliter_3[2];

                accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

                //������Ԫ��
                AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);//��̬����+�����˲�
                get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);
								KZ_Filter();

                //�����ǿ���У׼
                {
									
                    static uint16_t start_gyro_cali_time = 0;
                    if(start_gyro_cali_time == 0)
                    {
                        Gyro_Offset[0] = gyro_cali_offset[0];
                        Gyro_Offset[1] = gyro_cali_offset[1];
                        Gyro_Offset[2] = gyro_cali_offset[2];
                        start_gyro_cali_time++;
                    }
//                    else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
//                    {
//                        IMUWarnBuzzerOn();
//											
//                        if( first_temperate)
//                        {
//                            //������gyro_offset������������˶�start_gyro_cali_time++��������˶� start_gyro_cali_time = 0
//                            gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
//                        }
//                    }
//                    else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
//                    {

//                        IMUWarnBuzzerOFF();
//                        start_gyro_cali_time++;
//                    }
                }       
            }          
        }           
        //IMU_temp_Control(mpu6500_real_data.temp);
}


/**
  * @brief          MPU6500��IST8310��ʼ��
  * @author         Stone
  * @param[in]      void
  * @retval         void
  */
void IMU_Setup(void)
{
    //��ʼ��mpu6500��ʧ�ܽ�����ѭ��
		//SPI5 + DMA
    while (mpu6500_init() != MPU6500_NO_ERROR)
    {}

		//��ʼ��ist8310��ʧ�ܽ�����ѭ��
		//ģ��IIC
    while (ist8310_init() != IST8310_NO_ERROR)
    {}
			
   //��ʼ��mpu6500������׼�����ⲿ�ж�
    MPU6500_DATA_READY_EXIT_INIT();
	//��ʼ��SPI��DMA����ķ���
    MPU6500_SPI_DMA_Init(mpu6500_spi_DMA_txbuf, mpu6500_spi_rxbuf);
}


/**
  * @brief          У׼������
  * @author         RM
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[in]      �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  * @retval         ���ؿ�
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (first_temperate)
    {
        if( *time_count == 0)
        {
            Gyro_Offset[0] = gyro_cali_offset[0];
            Gyro_Offset[1] = gyro_cali_offset[1];
            Gyro_Offset[2] = gyro_cali_offset[2];
        }
        gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, time_count);

        cali_offset[0] = Gyro_Offset[0];
        cali_offset[1] = Gyro_Offset[1];
        cali_offset[2] = Gyro_Offset[2];
    }
}

/**
  * @brief          У׼���������ã�����flash���������ط�����У׼ֵ
  * @author         RM
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư
  * @retval         ���ؿ�
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
}


/**
  * @brief          ȡ�ýǶ�ֵָ��
  * @author         Stone
  * @param[in]      void
  * @retval         INS_Angle[]������Ԫ�ص�ַ
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_Angle;
}

/**
  * @brief          ȡ��������ֵָ��
  * @author         Stone
  * @param[in]      void
  * @retval         INS_gyro[]������Ԫ�ص�ַ
  */
const fp32 *get_MPU6500_Gyro_Data_Point(void)
{
    return INS_gyro;
}

/**
  * @brief          ȡ�ü��ٶȼ�ֵָ��
  * @author         Stone
  * @param[in]      void
  * @retval         INS_accel[]������Ԫ�ص�ַ
  */
const fp32 *get_MPU6500_Accel_Data_Point(void)
{
    return INS_accel;
}

/**
  * @brief          ��ȥ��Ư�Լ���ת����ϵ
  * @author         Stone
  * @param[in]      IMU9���ֵ
  * @retval         void
  */
//Gyro_Scale_Factor[i][0]������У׼���Զ�
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = mpu6500->gyro[0] * Gyro_Scale_Factor[i][0] + mpu6500->gyro[1] * Gyro_Scale_Factor[i][1] + mpu6500->gyro[2] * Gyro_Scale_Factor[i][2] 
									+ Gyro_Offset[i];
        accel[i] = mpu6500->accel[0] * Accel_Scale_Factor[i][0] + mpu6500->accel[1] * Accel_Scale_Factor[i][1] + mpu6500->accel[2] * Accel_Scale_Factor[i][2]
									+ Accel_Offset[i];
        mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2]
									+ Mag_Offset[i];
    }
}

/**
  * @brief          IMU���¿��ƣ����Ƴɱ�MCU���¶ȸ�10��
  * @author         Stone
  * @param[in]      IMU��ʵʱ�¶�
  * @retval         void
  */
static void IMU_temp_Control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0 ;
    if (first_temperate)
    {
        PID_Calc(&imuTempPid, temp, INS_GET_CONTROL_TEMPERATURE());
        if (imuTempPid.out < 0.0f)
        {
            imuTempPid.out = 0.0f;
        }
        tempPWM = (uint16_t)imuTempPid.out;
        IMUTempPWM(tempPWM);
    }
    else
    {
        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        if (temp > INS_GET_CONTROL_TEMPERATURE())
        {
            temp_constant_time ++;
            if(temp_constant_time > 200)
            {
                //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                first_temperate = 1;
                imuTempPid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;

            }
        }

        IMUTempPWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}
////////////////////////////////////////////
#if defined(MPU6500_USE_DATA_READY_EXIT)

void MPU6500_DATA_READY_EXIT_IRQHandler(void)
{
    if (EXTI_GetITStatus(MPU6500_DATA_READY_EXIT_Line) != RESET)
    {

        EXTI_ClearITPendingBit(MPU6500_DATA_READY_EXIT_Line);

//�������DMA���� ����������DMA�ж����
#if defined(MPU6500_USE_SPI_DMA)
        mpu6500_SPI_NS_L();
        MPU6500_SPI_DMA_Enable();
#endif
    }
}

#endif
////////////////////////////////////////////

#if defined(MPU6500_USE_SPI) && defined(MPU6500_USE_SPI_DMA)

void MPU6500_DMA_IRQHandler(void)
{
    if (DMA_GetFlagStatus(MPU6500_DMA_Stream, MPU6500_DMA_FLAG))
    {
        DMA_ClearFlag(MPU6500_DMA_Stream, MPU6500_DMA_FLAG);
        mpu6500_SPI_NS_H();
    }
}

#endif




