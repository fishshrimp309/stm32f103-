#include "control.h"
#include "motor.h"
#include "mpu6050.h"
#include "rc.h"
#include "pid.h"
#include "cmsis_os.h"
#include "rc.h"

int16_t Virtual_Throttle = 1000;//停转//1080min
int16_t m[4];

PID pid_roll_angle;//横滚轴
PID pid_roll_gyro;
PID pid_pitch_angle;//俯仰轴
PID pid_pitch_gyro;
PID pid_yaw_gyro;//偏航轴

void Control_PID_Init()
{
	PID_Init(&pid_roll_angle,3.5, 1, 0, 0.0f, 100.0f);//外环一般不加积分项//2
	PID_Init(&pid_roll_gyro,0.9, 0, 0, 50.0f, 250.0f);
	
	PID_Init(&pid_pitch_angle,3.5, 0.5, 0, 0.0f, 100.0f);
	PID_Init(&pid_pitch_gyro,0.9, 0, 0, 50.0f, 250.0f);
	
	PID_Init(&pid_yaw_gyro,3.5, 0, 0, 50.0f, 250.0f);
}

void Control_Mixer_Compute(int16_t throttle, float roll_out, float pitch_out, float yaw_out)
{
    m[0] = (int16_t)(throttle + roll_out - pitch_out + yaw_out + (throttle - 1000)*0.2); // 左前  
	m[1] = (int16_t)(throttle - roll_out - pitch_out - yaw_out); // 右前
    m[2] = (int16_t)(throttle + roll_out + pitch_out - yaw_out); // 右后
    m[3] = (int16_t)(throttle - roll_out + pitch_out + yaw_out); // 左后

	
	for(uint8_t i= 0;i < 4;i++)
	{
		if(m[i] < 1000) m[i] = 1000; 
		if(m[i] > 2000) m[i] = 2000;
	}

    if(rc.isStop_flag == 1)
    {
      Motor_Stop();
    }
    else
    {
      Motor_SetSpeed(m[0], m[1], m[2], m[3]);
    }
}


void Task_ControlCallback(float t_roll, float t_pitch, float t_yaw, int16_t throttle, float c_roll, float c_pitch, float c_yaw, float g_x, float g_y, float g_z)
{
    float Target_Gyro_Roll  = PID_Compute(&pid_roll_angle,  t_roll,  c_roll);
    float Target_Gyro_Pitch = PID_Compute(&pid_pitch_angle, t_pitch, c_pitch);
    float Target_Gyro_Yaw   = t_yaw; 

    float roll_out  = PID_Compute(&pid_roll_gyro,  Target_Gyro_Roll,  g_x);
    float pitch_out = PID_Compute(&pid_pitch_gyro, Target_Gyro_Pitch, g_y);
    float yaw_out   = PID_Compute(&pid_yaw_gyro,   Target_Gyro_Yaw,   g_z);
	
	if(throttle < 1250)
	{
		roll_out = 0;
		pitch_out = 0;
	}

	Control_Mixer_Compute(throttle, roll_out, pitch_out, yaw_out);
}

void OS_ControlCallback(void *argument)
{
  for(;;)
  {
	if(rc.isStop_flag == 1)
    {
		Final_Throttle = 1000;
		Target_Pitch = 0;
		Target_Roll = 0;
		Target_Yaw = 0;
    }
	Task_ControlCallback(Target_Roll, Target_Pitch, Target_Yaw, Final_Throttle, Roll, Pitch, Yaw, Gyro_X, Gyro_Y, Gyro_Z);
	osDelay(1);
  }
}
