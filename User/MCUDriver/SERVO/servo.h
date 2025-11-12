//
// Created by LENOVO on 2023/3/2.
//

#ifndef GIMBAL_2023_ABOARD_SERVO_H
#define GIMBAL_2023_ABOARD_SERVO_H
#include "stm32f4xx_hal.h"
void Servo_on(void);
void Servo_off(void);
void Servo_StartPWM(void);
int8_t Servo_GetStatus(void);
#endif //GIMBAL_2023_ABOARD_SERVO_H
