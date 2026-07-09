#ifndef __PID_H
#define __PID_H

#include "main.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float target;
  float measure;
  float err[2];   // 0,error 1, last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;

void PID_Init(pid_struct_t *pid,float kp,float ki,float kd,float i_max,float out_max);
float  PID_Compute(pid_struct_t *pid, float target, float measure);

#endif
