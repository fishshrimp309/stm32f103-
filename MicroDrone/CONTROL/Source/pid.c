#include "pid.h"

void PID_Init(pid_struct_t *pid,float kp,float ki,float kd,float i_max,float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

float PID_Compute(pid_struct_t *pid, float target, float measure)
{
  pid->target = target;
  pid->measure = measure;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->target - pid->measure;
  
  pid->p_out  = pid->kp * pid->err[0];
	
  pid->i_out += pid->ki * pid->err[0];
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
	
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
