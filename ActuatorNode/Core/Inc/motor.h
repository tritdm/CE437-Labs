#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#define encoderResolution 580.0f // số xung thay đổi khi xoay 1 vòng
#define PI 3.14159265359f
#define diameter 13.5f //cm
#define CIRCUMFERENCE_OF_WHEEL (PI * (diameter))
typedef struct{
  int16_t encodeCnt;
  int16_t preEncoderCnt;

  int16_t position;
  int16_t prePosition;

  int timeIndex;
  float numRoundPerSec;
  float speed; // cm/s

} encoderMotor;

typedef enum{
	STOP,
	START,
	FORWARD,
	REVERSE,
}bts_state; //BTS7960

typedef struct{
	TIM_HandleTypeDef* 		Timer_Handle;
	uint16_t				Timer_Channel_L;
	uint16_t				Timer_Channel_R;

	GPIO_TypeDef*			En_L_GPIOx;
	uint16_t				En_L_GPIO_Pin;

	GPIO_TypeDef*			En_R_GPIOx;
	uint16_t				En_R_GPIO_Pin;

	struct{
		uint16_t durum;
	}bts_set;

}bts7960_config;

/**
  * @brief  Call function HAL_TIM_Encoder_Start_IT
  * @retval void
  */
void startEncoder();
void updateEncoder();
void bts7960_init (	bts7960_config* bts7960_config,
					TIM_HandleTypeDef* 		Timer_Handle,
					uint16_t				Timer_Channel_L,
					uint16_t				Timer_Channel_R,
					GPIO_TypeDef*			En_L_GPIOx,
					uint16_t				En_L_GPIO_Pin,
					GPIO_TypeDef*			En_R_GPIOx,
					uint16_t				En_R_GPIO_Pin);

//void bts_start(bts7960_config* bts7960_config, uint16_t state);
uint8_t scalePWM(uint8_t PWM); // giới hạn PWM từ 0-100
float velocityToPWM(float velocity);
void startMotor(bts7960_config* bts7960_config);
void stopMotor(bts7960_config* bts7960_config);
void goForward(bts7960_config* bts7960_config, uint8_t PWM);
void goReverse(bts7960_config* bts7960_config, uint8_t PWM);
#endif /* __MOTOR_H__ */
