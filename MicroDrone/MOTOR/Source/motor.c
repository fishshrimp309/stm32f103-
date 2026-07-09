#include "motor.h"

void Motor_Init()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Motor_SetSpeed(float left_front, float right_front, float left_rear, float right_rear)
{
	int motor[4];
	motor[0] = (int)left_front;
	motor[1] = (int)right_front;
	motor[2] = (int)left_rear;
	motor[3] = (int)right_rear;

	for(uint8_t i=0;i<4;i++)
	{
		if(motor[i] < 1000) motor[i] = 1000;
		if(motor[i] > 2000) motor[i] = 2000;
	}
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor[0]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor[1]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor[2]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor[3]);
}
