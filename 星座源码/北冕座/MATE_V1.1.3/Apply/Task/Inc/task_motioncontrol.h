
#ifndef __TASKMOTIONCONTROL_H_
#define __TASKMOTIONCONTROL_H_

#define MAX_HPOWER 200
#define MIN_HPOWER 50
#define MAX_VPOWER 200
#define YAWSPEED 180
#define SLOW_SPEED 40

void Task_Motion_Process(void);
void HandleMode_data_handle(float ExpDepth);
void PWNOutput_limit(void);
void Thruster_nagative_data_handle(float times_L,float times_R);
void Balance_data_handle(void);

#endif
