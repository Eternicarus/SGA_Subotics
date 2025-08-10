#ifndef __TASK_SERVOCONTROL_H_
#define __TASK_SERVOCONTROL_H_

void Servo_Write(PWMInfo_T *t_PWM);
void Servo_Init(PWMInfo_T *t_PWM);
void Servo_Limit(PWMInfo_T *t_PWM);

#endif
