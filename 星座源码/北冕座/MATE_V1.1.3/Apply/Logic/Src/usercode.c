#include "usercode.h"		/* usercode头文件 */
#include "drv_hal_conf.h"   /* SGA库头文件配置 */
#include "task_conf.h"      /* task层头文件配置 */
#include "ocd_conf.h"       /* OCD层头文件配置 */
#include "dev_conf.h"		/* Dev层头文件配置 */
#include "algo_conf.h"		/* Algo层头文件配置 */
#include "config.h"			/* I/O配置头文件配置 */

uint8_t num;
uint8_t RxData[100];

/* 线程句柄 */
rt_thread_t thread1 = RT_NULL;
rt_thread_t thread2 = RT_NULL;
rt_thread_t thread3 = RT_NULL;
rt_thread_t thread4 = RT_NULL;
rt_thread_t thread5 = RT_NULL;
rt_thread_t thread6 = RT_NULL;
rt_thread_t thread7 = RT_NULL;
rt_thread_t thread8 = RT_NULL;
rt_thread_t thread9 = RT_NULL;
rt_thread_t thread10 = RT_NULL;
rt_thread_t thread11 = RT_NULL;

/* 信号量句柄*/
rt_sem_t DataFromIPC_Sem = RT_NULL;	//上位机数据信号量
rt_sem_t JY901S_Sem = RT_NULL;		//JY901S数据信号量

/*消息队列句柄*/
rt_mq_t AutoModemq = RT_NULL;		//自动模式消息队列
rt_mq_t HandleModemq = RT_NULL;		//手动模式消息队列
rt_mq_t DepthControlmq = RT_NULL;	//定深数据消息队列

//wdg
static rt_timer_t watchdog_timer = RT_NULL;
static void watchdog_timer_callback(void* parameter)
{
    Drv_IWDG_Feed(&demoIWDG);
}

// ALIGN(RT_ALIGN_SIZE)
/* 用户逻辑代码 */
void UserLogic_Code(void)
{
	/* 创建信号量 */
	JY901S_Sem = rt_sem_create("JY901Sem",0,RT_IPC_FLAG_FIFO);
	//if(RT_NULL != JY901S_Sem)	printf("JY901Sem create successful\r\n");

	DataFromIPC_Sem = rt_sem_create("DataFromIPC_Sem",0,RT_IPC_FLAG_FIFO);
	//if(RT_NULL != DataFromIPC_Sem)	printf("DataFromIPC_Sem create successful\r\n");

	/* 创建消息队列 */
	AutoModemq = rt_mq_create("AutoModemq",50,50,RT_IPC_FLAG_FIFO);
	//if(RT_NULL != AutoModemq)		printf("AutoModemq create successful\r\n");
	HandleModemq = rt_mq_create("HandleModemq",50,10,RT_IPC_FLAG_FIFO);
	//if(RT_NULL != HandleModemq)		printf("HandleModemq create successful\r\n");
	DepthControlmq = rt_mq_create("DepthControlmq",50,10,RT_IPC_FLAG_FIFO);
	//if(RT_NULL != DepthControlmq)		printf("DepthControlmq create successful\r\n");

	/* 
		线程1 上位机数据读取
		线程2 JY901S数据读取
		线程3 MS5837数据读取
		线程4 手柄运动模式
		线程5 自动巡线模式
		线程6 运动控制
		线程7 定深控制
		线程8 缓增控制
		线程9 汇报当前PWM输出
		线程10 PH反馈
		线程11 测试线程
	 */

  


	/* 创建线程 */
	thread1 = rt_thread_create("DataFromIPC",DataFromIPC,NULL,1024,1,20);
	// if(RT_NULL != thread1)
	// 	printf("DataFromIPC create successful\r\n");

	thread2 = rt_thread_create("JY901SReadThread",JY901SReadThread,NULL,512,4,20);
	// if(RT_NULL != thread2)
	// 	printf("JY901SReadThread create successful\r\n");


	thread3 = rt_thread_create("MS5837ReadThread",MS5837ReadThread,NULL,512,4,20);
	// if(RT_NULL != thread3)
	// 	printf("MS5837ReadThread create successful\r\n");	


	thread4 = rt_thread_create("MODE_HANDLE",HANDLE_MODE,NULL,512,3,40);
	// if(RT_NULL != thread4)
	// 	printf("HANDLE_MODE create successful\r\n");
	

	thread5 = rt_thread_create("MODE_AUTO",AUTO_MODE,NULL,512,7,40);
	// if(RT_NULL != thread5)
	// 	printf("AUTO_MODE create successful\r\n");

	thread6 = rt_thread_create("MotionControl",MotionControl,NULL,1024,2,40);
	// if(RT_NULL != thread6)
	// 	printf("DepthControl create successful\r\n");

	thread7 = rt_thread_create("DepthControl",DepthControl,NULL,512,3,20);
	// if(RT_NULL != thread7)
	// 	printf("DepthControl create successful\r\n");

	thread8 = rt_thread_create("PlusControl",PlusControl,NULL,512,2,20);
	// if(RT_NULL != thread8)
	// 	printf("ServoControl create successful\r\n");

	thread9 = rt_thread_create("ReportPWMout",ReportPWMout,NULL,512,7,20);
	// if(RT_NULL != thread7)
	// 	printf("ReportPWMout create successful\r\n");

	thread10 = rt_thread_create("pH_outcome",pH_outcome,NULL,512,3,20);
	// if(RT_NULL != thread7)
	// 	printf("TestThread create successful\r\n");
	thread11 = rt_thread_create("TestThread",TestThread,NULL,256,7,20);

	rt_thread_startup(thread1);		//线程1 上位机数据读取
	rt_thread_startup(thread2);		//线程2 JY901S数据读取
	rt_thread_startup(thread3);		//线程3 MS5837数据读取
	rt_thread_startup(thread4);		//线程4 手柄运动模式
	rt_thread_startup(thread5);		//线程5 自动巡线模式
	rt_thread_startup(thread6);		//线程6 运动控制
	rt_thread_startup(thread7);		//线程7 定深控制
	rt_thread_startup(thread8);		//线程8 缓增控制
	rt_thread_startup(thread9);		//线程9 汇报当前PWM输出
	rt_thread_startup(thread10);	//线程10 PH反馈
	rt_thread_startup(thread11);    //线程11 测试线程
watchdog_timer = rt_timer_create("watchdog", watchdog_timer_callback, RT_NULL, rt_tick_from_millisecond(1000), RT_TIMER_FLAG_PERIODIC);
    if (watchdog_timer != RT_NULL)
    {
        rt_timer_start(watchdog_timer);
    }
	//打印现在深度PID的值
	printf("P:%f\n", DepthPID.fKp);
    printf("I:%f\n", DepthPID.fKi);
    printf("D:%f\n", DepthPID.fKd);
	
}


