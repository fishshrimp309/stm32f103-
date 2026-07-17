#include "control.h"

int16_t Virtual_Throttle = 1000;//停转

pid_struct_t pid_roll_angle;//横滚轴
pid_struct_t pid_roll_gyro;
pid_struct_t pid_pitch_angle;//俯仰轴
pid_struct_t pid_pitch_gyro;
pid_struct_t pid_yaw_gyro;//偏航轴

void Control_PID_Init()
{
	PID_Init(&pid_roll_angle,1, 0, 0, 0.0f, 150.0f);//角度环一般不加积分项
	PID_Init(&pid_roll_gyro,-1, 0, 0, 50.0f, 250.0f);
	
	PID_Init(&pid_pitch_angle,0, 0, 0, 0.0f, 150.0f);
	PID_Init(&pid_pitch_gyro,0, 0, 0, 50.0f, 250.0f);
	
	PID_Init(&pid_yaw_gyro,0, 0, 0, 50.0f, 250.0f);
}

void Control_Mixer_Compute(int16_t throttle, float roll_out, float pitch_out, float yaw_out)
{
    int16_t m1 = (int16_t)(throttle + roll_out + pitch_out - yaw_out); // 左前  
	int16_t m2 = (int16_t)(throttle - roll_out + pitch_out + yaw_out); // 右前
    int16_t m3 = (int16_t)(throttle - roll_out - pitch_out - yaw_out); // 右后
    int16_t m4 = (int16_t)(throttle + roll_out - pitch_out + yaw_out); // 左后

    if(m1 < 1000) m1 = 1000; if(m1 > 2000) m1 = 2000;
    if(m2 < 1000) m2 = 1000; if(m2 > 2000) m2 = 2000;
    if(m3 < 1000) m3 = 1000; if(m3 > 2000) m3 = 2000;
    if(m4 < 1000) m4 = 1000; if(m4 > 2000) m4 = 2000;

    Motor_SetSpeed(m1, m2, m3, m4);
}


void Control_Flight_Loop(float t_roll, float t_pitch, float t_yaw, int16_t throttle,
                         float c_roll, float c_pitch, float c_yaw,
                         float g_x, float g_y, float g_z)
{
    float Target_Gyro_Roll  = PID_Compute(&pid_roll_angle,  t_roll,  c_roll);
    float Target_Gyro_Pitch = PID_Compute(&pid_pitch_angle, t_pitch, c_pitch);
    float Target_Gyro_Yaw   = t_yaw; 

    float roll_out  = PID_Compute(&pid_roll_gyro,  Target_Gyro_Roll,  g_x);
    float pitch_out = PID_Compute(&pid_pitch_gyro, Target_Gyro_Pitch, g_y);
    float yaw_out   = PID_Compute(&pid_yaw_gyro,   Target_Gyro_Yaw,   g_z);

	Control_Mixer_Compute(throttle, roll_out, pitch_out, yaw_out);
}
