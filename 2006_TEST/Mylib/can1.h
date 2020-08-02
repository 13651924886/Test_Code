#ifndef __CAN2_H__
#define __CAN2_H__
#define PWM_MODE                                0x01
#define PWM_CURRENT_MODE                        0x02
#define PWM_VELOCITY_MODE                       0x03
#define PWM_POSITION_MODE                       0x04
#define PWM_VELOCITY_POSITION_MODE              0x05
#define CURRENT_VELOCITY_MODE                   0x06
#define CURRENT_POSITION_MODE                   0x07
#define CURRENT_VELOCITY_POSITION_MODE          0x08
#define CANx                       	CAN1
#define CAN_CLK                     RCC_APB1Periph_CAN1
#define CAN_RX_IRQ									CAN1_RX0_IRQn
#define CAN_RX_IRQHandler			  		CAN1_RX0_IRQHandler

#define CAN_RX_PIN                 GPIO_Pin_0
#define CAN_TX_PIN                 GPIO_Pin_1
#define CAN_TX_GPIO_PORT           GPIOD
#define CAN_RX_GPIO_PORT           GPIOD
#define CAN_TX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define CAN_RX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define CAN_AF_PORT                GPIO_AF_CAN1
#define CAN_RX_SOURCE              GPIO_PinSource0
#define CAN_TX_SOURCE              GPIO_PinSource1
void CAN1_Configuration(void);

extern short Real_Current_Value[4];
extern short Real_Velocity_Value[4];
extern long Real_Position_Value[4];
extern char Real_Online[4];
extern char Real_Ctl1_Value[4];
extern char Real_Ctl2_Value[4];

extern short PitchAngle;
extern short PitchCurrentFeedback;
extern short PitchCurrentWanted;
extern short PitchCurrentRealV;
extern short PitchRealV;

extern short YawAngle;
extern short YawCurrentFeedback;
extern short YawCurrentWanted;
extern short YawCurrentRealV;
extern short YawRealV;

#endif 
