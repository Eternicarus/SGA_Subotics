#ifndef __CONFIG_H_
#define __CONFIG_H_

#include "drv_hal_conf.h"
#include "ocd_conf.h"
#include "algo_conf.h"
#include "dev_conf.h"

//????????????
typedef struct
{
	float Roll;
	float Pitch;
	float Yaw;
	
}Expect_angle;

/* 用户句柄声明包含区 */

extern volatile char x_y_z_pitch;

extern volatile char left_rocker;

extern volatile char right_rocker;

extern volatile char Mode_control;

extern volatile char speed_kH;

extern volatile char speed_kV;

extern volatile char DepthFlag;

extern volatile char SpeedMode;

extern volatile char cnt;

extern volatile float PIDOut;

extern volatile float DepthPID_out;

extern volatile float PitchPID_out;

extern volatile float DepthFliter;

extern volatile char PitchFlag;

extern volatile char floatflag;

extern volatile char sinkflag;

extern volatile char pHflag;

extern Expect_angle Exp_AngleInfo;

extern tagGPIO_T demoGPIO[];

extern tagUART_T Uart1;

extern tagUART_T Uart3;

extern tagJY901_T JY901S;

extern tagPWM_T PWM[];

extern tagMS5837_T MS5837;

extern tagPID_T DepthPID;

extern tagPID_T YawPID;

extern tagPID_T RollPID;

extern tagPID_T PitchPID;

extern tagPID_T LinePatrolPID;

extern tagPID_T BalancePID;

extern tagPID_T AngleLoopPID;

extern tagPID_T PositionLoopPID;

extern tagIWDG_T demoIWDG;
 
extern tagADC_T pH_adc;

#endif
