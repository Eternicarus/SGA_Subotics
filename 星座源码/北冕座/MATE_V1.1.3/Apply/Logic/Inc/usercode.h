#ifndef	__USERCODE_H_
#define	__USERCODE_H_

#include "drv_hal_conf.h"

void UserLogic_Code(void);

extern rt_thread_t thread1;
extern rt_thread_t thread2;
extern rt_thread_t thread3;
extern rt_thread_t thread4;
extern rt_thread_t thread5;
extern rt_thread_t thread6;
extern rt_thread_t thread7;
extern rt_thread_t thread8;
extern rt_thread_t thread9;
extern rt_thread_t thread10;
extern rt_thread_t thread11;

extern rt_sem_t JY901S_Sem;
extern rt_sem_t DataFromIPC_Sem;

extern rt_mq_t AutoModemq;
extern rt_mq_t HandleModemq;
extern rt_mq_t DepthControlmq;

#endif
