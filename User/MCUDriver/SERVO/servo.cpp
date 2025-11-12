//
// Created by LENOVO on 2023/3/2.
//
#include "servo.h"
#include "gimbalc.h"
#include "tim.h"

extern TIM_HandleTypeDef htim8;
static int8_t servo_status;
void Servo_StartPWM(void)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 10);
	servo_status = SERVO_OFF;
}

void  Servo_off(void)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 19); //¶æ»ú×ª90¡ã
	servo_status = SERVO_ON;
}

void Servo_on(void)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 10);
	servo_status = SERVO_OFF;
}

int8_t Servo_GetStatus(void)
{
	return servo_status;
}