#ifndef __TASK_THRUSTER_H_
#define	__TASK_THRUSTER_H_

#define STOP_PWM_VALUE 1500 //停转高电平时间（ns）
#define STOP_PWM_VALUE_F 1525 //停转高电平时间（ns）
#define STOP_PWM_VALUE_B 1465 //停转高电平时间（ns）
#define DEADBAND        0  //死区范围（ns）

#define CLAW_CATCH_STOP_ANGLE 180 //减为抓紧，最大张角接近110，尽量小15到20度比较适合
#define CLAW_ELBOW_STOP_ANGLE 120
#define CLAW_SHOUDER_STOP_ANGLE 170

#define CLAW_STEP 7


void Task_Thruster_Init(void);
void Task_Thruster_SpeedSet(int index,uint16_t _HTime);
void Task_Thruster_AllSpeedSet(uint16_t _HTime);
void Task_Thruster_Start(int index,uint16_t _HTime);
void Task_Thruster_AllStart(uint16_t *adress);
void Task_Thruster_Stop(int index);
void Task_Thruster_AllStop(void);
uint16_t Servo_Angle_To_HightTime(uint16_t angle);
void Task_Servo_AllStart(uint16_t *adress);
void Task_Thruster_pin_AllStart(uint16_t *adress);

#endif
