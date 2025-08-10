#ifndef _SQRT_P_CONTROL_H_
#define _SQRT_P_CONTROL_H_
float sign(float x);
float safe_sqrt(float x);
float sqrt_controller(float error, tagPID_T *_tPID);
#endif
