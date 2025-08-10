#ifndef __THREADPOOL_H_
#define __THREADPOOL_H_

#include "stdint.h"

//推进器宏定义
#define v_wheel1_speed   	3       //水平推进器   E9 右后
#define v_wheel2_speed   	1       //水平推进器   E13 右前
#define v_wheel3_speed   	0       //水平推进器   E11 左前
#define v_wheel4_speed   	2       //水平推进器   E14 左后
		
#define h_wheel1_speed   	8        //左前垂直推进器B4
#define h_wheel2_speed   	9        //右前垂直推进器B5
#define h_wheel3_speed   	6        //左后垂直推进器B0
#define h_wheel4_speed   	11       //右后垂直推进器B1

#define claw_servo_elbow	 4       //机械爪小臂旋转舵机B6
#define claw_servo_down  	 13       //下机械爪抓取舵机B7
#define claw_servo_gimbal    10      //摄像头云台C8
#define claw_servo_up  		 7       //上机械爪抓取舵机B8
#define claw_servo_shoulder  12       //大臂舵机
#define claw_servo_syringe   5       //注射器推进舵机B9
//PWM结构体
typedef struct
{
    uint16_t PWMout[14];
}PWMInfo_T;

/* 函数声明 */
void DataFromIPC(void* paramenter);
void JY901SReadThread(void* paramenter);
void circle_conduct(void);
void MS5837ReadThread(void* paramenter);
void HANDLE_MODE(void* paramenter);
void AUTO_MODE(void* paramenter);
void DepthControl(void* paramenter);
void PlusControl(void* paramenter);
void ReportPWMout(void* paramenter);
void TestThread(void* paramenter);
void MotionControl(void* paramenter);
void pH_outcome(void* paramenter);
#endif
