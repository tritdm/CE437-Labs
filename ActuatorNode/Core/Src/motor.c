#include "motor.h"

extern TIM_HandleTypeDef htim2;
extern encoderMotor encoderInfo;
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == htim2.Instance)
//	{
//		timeElapsed += 100;
//		if (isButtonPressed) pressTime += 100;
//	}
//	else
//	{
//		__NOP();
//	}
//}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim2.Instance)
		{
		encoderInfo.encodeCnt = (int16_t)__HAL_TIM_GET_COUNTER(htim);
		encoderInfo.position = encoderInfo.encodeCnt/encoderResolution;
		}
		else
		{
			__NOP();
		}
//	counter = __HAL_TIM_GET_COUNTER(htim);
//
//	count = (int16_t)counter;
//
//	position = count/4;
}
void startEncoder()
{
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
}
void bts7960_init (		bts7960_config* bts7960_config,
						TIM_HandleTypeDef* 		Timer_Handle,
						uint16_t				Timer_Channel_L,
						uint16_t				Timer_Channel_R,
						GPIO_TypeDef*			En_L_GPIOx,
						uint16_t				En_L_GPIO_Pin,
						GPIO_TypeDef*			En_R_GPIOx,
						uint16_t				En_R_GPIO_Pin)
{
	bts7960_config->Timer_Handle	= Timer_Handle;
	bts7960_config->Timer_Channel_L  	= Timer_Channel_L;
	bts7960_config->Timer_Channel_R  	= Timer_Channel_R;

	bts7960_config->En_L_GPIOx		= En_L_GPIOx;
	bts7960_config->En_L_GPIO_Pin	= En_L_GPIO_Pin;

	bts7960_config->En_R_GPIOx		= En_R_GPIOx;
	bts7960_config->En_R_GPIO_Pin	= En_R_GPIO_Pin;
}

//void bts_start(bts7960_config* bts7960_config, uint16_t state){
//
//	if(state == START){
//		bts7960_config->bts_set.durum = state;
//		HAL_GPIO_WritePin(bts7960_config->En_L_GPIOx, bts7960_config->En_L_GPIO_Pin, GPIO_PIN_SET);
//		HAL_TIM_PWM_Start_IT(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel);
//		__HAL_TIM_SET_COMPARE(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel, 200);
//
//	}
//	else{
//		bts7960_config->bts_set.durum = state;
//		HAL_GPIO_WritePin(bts7960_config->En_L_GPIOx, bts7960_config->En_L_GPIO_Pin, GPIO_PIN_RESET);
//		HAL_TIM_PWM_Stop_IT(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_L);
//		HAL_TIM_PWM_Stop_IT(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_R);
//
//	}
//}
void startMotor(bts7960_config* bts7960_config)
{
	bts7960_config->bts_set.durum = START;
	HAL_GPIO_WritePin(bts7960_config->En_L_GPIOx, bts7960_config->En_L_GPIO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(bts7960_config->En_R_GPIOx, bts7960_config->En_R_GPIO_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start_IT(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_L);
	HAL_TIM_PWM_Start_IT(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_R);
	__HAL_TIM_SET_COMPARE(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_L, 0);
	__HAL_TIM_SET_COMPARE(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_R, 100);
}

void stopMotor(bts7960_config* bts7960_config)
{
	bts7960_config->bts_set.durum = STOP;
	HAL_GPIO_WritePin(bts7960_config->En_L_GPIOx, bts7960_config->En_L_GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(bts7960_config->En_R_GPIOx, bts7960_config->En_R_GPIO_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop_IT(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_L);
	HAL_TIM_PWM_Stop_IT(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_R);
}

void goForward(bts7960_config* bts7960_config, uint16_t PWM )
{
	bts7960_config->bts_set.durum = FORWARD;
	__HAL_TIM_SET_COMPARE(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_L, PWM);
	__HAL_TIM_SET_COMPARE(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_R, 0);
}

void goReverse(bts7960_config* bts7960_config, uint16_t PWM)
{
	bts7960_config->bts_set.durum = REVERSE;
	__HAL_TIM_SET_COMPARE(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_L, 0);
	__HAL_TIM_SET_COMPARE(bts7960_config->Timer_Handle, bts7960_config->Timer_Channel_R, PWM);
}
