#ifndef __THREADPOOL_H_
#define __THREADPOOL_H_

#include "stdint.h"

//推进器宏定义
#define LeftHThruster   2       //?????????
#define RightHThruster  1       //?????????
#define LeftVThruster   0       //????????
#define RightVThruster  3       //?????????

#define v_wheel1_speed   3       //水平推进器   黑C9 右后
#define v_wheel2_speed   1       //水平推进器   灰C8 右前
#define v_wheel3_speed   2       //水平推进器   白B1 左后
#define v_wheel4_speed   0       //水平推进器   紫B0 左前

#define h_wheel1_speed   4       //左前垂直推进器B6
#define h_wheel2_speed   5       //右前垂直推进器B7
#define h_wheel3_speed   6       //左后垂直推进器B8
#define h_wheel4_speed   7       //右后垂直推进器B9

#define claw_shouder_Speed    9       //机械爪大臂旋转舵机E9
#define claw_elbow_Speed   8       //机械爪小臂旋转舵机E13
#define claw_catchSpeed  11      //机械爪抓取舵机E14
#define light_level_1  10      //灯光亮度

//PWM锟芥储锟结构锟斤拷
typedef struct
{
    uint16_t PWMout[12];
}PWMInfo_T;

/* 锟竭筹拷锟斤拷诤锟斤拷锟斤拷锟斤拷锟? */
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

#endif
