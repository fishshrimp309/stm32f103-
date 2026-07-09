#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

void Motor_Init(void);
void Motor_SetSpeed(float left_front, float right_front, float left_rear, float right_rear);

#endif
