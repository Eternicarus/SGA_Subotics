#ifndef __TASKMOTIONCONTROL_H_
#define __TASKMOTIONCONTROL_H_

#define MAX_HPOWER 180
#define MIN_HPOWER 50
#define MAX_VPOWER 200
#define YAWSPEED 90

void Task_Motion_Process(void);
void HandleMode_data_handle(void);
void PWNOutput_limit(void);
void Thruster_nagative_data_handle(float k);
void Balance_data_handle(void);

#endif
