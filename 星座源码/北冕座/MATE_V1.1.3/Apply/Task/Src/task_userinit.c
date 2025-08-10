#include "task_conf.h"
#include "ocd_conf.h"
#include "usercode.h"
#include "config.h"


/* 外设初始化函数 */
void Task_UserInit(void)
{   
    //初始化串口1和串口3
    Drv_Uart_DMAInit(&Uart1);
    Drv_Uart_DMAInit(&Uart3);

    //初始化PID
    Algo_PID_Init(&DepthPID);
    Algo_PID_Init(&YawPID);
    Algo_PID_Init(&AngleLoopPID);
    Algo_PID_Init(&PositionLoopPID);
    Algo_PID_Init(&RollPID);

    //初始化JY901S
    OCD_JY901_DMAInit(&JY901S);
    //printf("JY901S INIT!\r\n");

    //姿态初始化
    // Expect_angle_Init();

    //初始化MS5837
     if(!OCD_MS5837_Init(&MS5837))
         printf("MS5837 ERROR\r\n");

    OCD_MS5837_Init(&MS5837);

    //初始化PWM
    Drv_PWM_Init(PWM,14);
    //printf("PWM INIT!\r\n");
    
    //推进器上电初始化
    Task_Thruster_Init();
	
     //wdg
    Drv_IWDG_Init(&demoIWDG);// 防止跑飞 
    
    Drv_ADC_Init(&pH_adc,1);

}
