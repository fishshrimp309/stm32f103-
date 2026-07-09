#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "motor.h"
#include "pid.h"

void Control_PID_Init(void);
void Control_Mixer_Compute(int16_t throttle, float roll_out, float pitch_out, float yaw_out);
void Control_Flight_Loop(float t_roll, float t_pitch, float t_yaw, int16_t throttle,float c_roll, float c_pitch, float c_yaw,float g_x, float g_y, float g_z);

#endif
