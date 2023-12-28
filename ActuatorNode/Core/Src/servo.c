#include "servo.h"
extern TIM_HandleTypeDef htim4;
void initServo()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}
void setAngle(uint8_t Angle)
{
	if (Angle < 0)
	{
		Angle = 0;
	}
	else if (Angle > 90)
	{
		Angle = 90;
	}

	// Ánh xạ giá trị từ khoảng 0-90 sang khoảng 99-249
	uint8_t  output = 99 + (Angle * (249 - 99)) / 90;
	htim4.Instance->CCR1 = output;
}
uint16_t getAngle()
{
	return htim4.Instance->CCR1;
}
