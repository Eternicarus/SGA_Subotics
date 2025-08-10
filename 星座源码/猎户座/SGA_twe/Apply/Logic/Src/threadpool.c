#include "usercode.h"		/* usercode头文件 */
#include "threadpool.h"		/* threadpool头文件 */
#include "drv_hal_conf.h"   /* SGA库头文件配置 */
#include "task_conf.h"      /* task层头文件配置 */
#include "ocd_conf.h"       /* OCD层头文件配置 */
#include "dev_conf.h"		/* Dev层头文件配置 */
#include "algo_conf.h"		/* Algo层头文件配置 */
#include "config.h"			/* I/O配置头文件配置 */

//存储8个推进器的数据和3个舵机和探照灯的数据
PWMInfo_T PWMInfo = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1100};

/* 绾跨▼鍏ュ彛鍑芥暟锛堜娇鐢ㄨ８鏈哄拷鐣ワ拷?锟芥枃浠讹級 */
/* 读取上位机数据线程 */
void DataFromIPC(void* paramenter)
{
    uint8_t ReceBuf[100];
    uint8_t ReceNum = 0;

    while(1)
    {
        if(rt_sem_take(DataFromIPC_Sem,RT_WAITING_FOREVER) == RT_EOK)
        {
            //平时调试使用串口3，正式使用采用串口1,修改PRINTF_UART宏即可
            if(PRINTF_UART == USART3)
                ReceNum = Drv_Uart_Receive_DMA(&Uart3,ReceBuf);
            else if(PRINTF_UART == USART1)
                ReceNum = Drv_Uart_Receive_DMA(&Uart1,ReceBuf);
            
            if(ReceNum != 0)
            
            {
                //printf("%s\r\n",ReceBuf);
                //分析数据
                Task_AnalysisData(ReceBuf);

                //将临时变量清0
                rt_memset(ReceBuf,0,ReceNum);
                ReceNum = 0;
            }
        }
        Drv_Delay_Ms(1);    //让出CPU资源给低优先级线程
    }
}

/* 读取JY901S数据线程 */
void JY901SReadThread(void* paramenter)
{
    while(1)
    {
        //如果获取到信号量，说明接收到数据
		if(rt_sem_take(JY901S_Sem,RT_WAITING_FOREVER) == RT_EOK)
		{
            //如果成功处理完成数据
			if(OCD_JY901_DataProcess(&JY901S))
            {
                //数据转换
                OCD_JY901_DataConversion(&JY901S);
                //打印角速度和加速度
                OCD_JY901_Printf(&JY901S);
				//打印横滚角和俯仰角
				// if(JY901S.tConfig.usType & JY901_OUTPUT_ANGLE)	    
                    // printf("J Angle %.3f\r\n",
                    //         JY901S.stcAngle.ConPitch);
                //打印艄向角
                // printf("concon_YAW %.3f\r\n",concon_YAW);
				// printf("\r\n");
            }
		}
        Drv_Delay_Ms(1);    //璁╁嚭CPU璧勬簮缁欎綆浼樺厛绾х嚎锟�?
		rt_thread_yield();
    }
}

/* 读取MS5837数据线程▼ */
void MS5837ReadThread(void* paramenter)
{
    DepthControlInfo DCInfo;
    while(1)
    {
        OCD_MS5837_GetData(&MS5837);
        if(MS5837.fDepth == 153150.250000)  //未接MS5837的错误数据
            MS5837.fDepth = 0;
        //printf("M %0.2f\r\n",MS5837.fDepth);
        // printf("T %0.2f\r\n",MS5837.fTemperature);
        if(cnt == 0)
        {
            cnt = 1;
            DCInfo.setDepth = MS5837.fDepth;
            rt_mq_send(DepthControlmq,&DCInfo,sizeof(DepthControlInfo));
        }
        Drv_Delay_Ms(600);
    }
}

/* 手柄控制线程 */
void HANDLE_MODE(void* paramenter)
{
    HandleModeInfo HMInfo;
    AutoModeInfo ClearBuf;

    // rt_thread_suspend(rt_thread_self());
    // rt_schedule();
    while(1)
    {
        if(rt_mq_recv(HandleModemq,&HMInfo,sizeof(HandleModeInfo),RT_WAITING_NO) == RT_EOK)
        {
            //鎵嬫焺鎺у埗娑堟伅闃熷垪鎺ユ敹鍒版暟锟�?锛屽皢鍒囨崲鍒拌嚜鍔ㄦā锟�?
            if(!rt_strcmp(HMInfo.ModeChange,"AUTO START"))
            {
                //灏咥utoModemq娑堟伅闃熷垪锟�?鎵€鏈夊唴瀹规竻锟�?
                while(1)
                {
                    if(rt_mq_recv(AutoModemq,&ClearBuf,sizeof(AutoModeInfo),RT_WAITING_NO) != RT_EOK)
                        break;
                }
                Task_Thruster_AllStop();                //鎵€鏈夋帹杩涘櫒鍋滆浆
                
                printf("Switch to AUTO Mode\r\n");
                rt_enter_critical();                    //璋冨害鍣ㄤ笂锟�?
                rt_thread_suspend(rt_thread_self());    //鎸傝捣锟�?绾跨▼
                rt_thread_resume(thread5);              //鎭�锟�?锟借嚜鍔ㄦ帶鍒剁嚎锟�?
                rt_exit_critical();                     //璋冨害鍣ㄨВ锟�?
                rt_schedule();                          //绔嬪嵆鎵э拷?锟戒竴娆¤皟锟�?
            }
            else
            {
                Task_HandleMode_Process(HMInfo);    //鎵嬫焺鎺у埗妯″紡澶勭悊鍑芥暟
                Drv_Delay_Ms(500);  //鎵э拷?锟芥椂闂翠笌涓婁綅鏈烘墜鏌勫彂閫佷竴甯ф暟锟�?鏃堕棿鐩稿悓
            }
        }
        // printf("HANDLE\r\n");

        Drv_Delay_Ms(1);    //璁╁嚭CPU璧勬簮缁欎綆浼樺厛绾х嚎锟�?
    }
}

/* 运动控制主线程 */
void MotionControl(void* paramenter)
{
    while(1)
    {
		Task_Motion_Process();
        Drv_Delay_Ms(100);
    }
}


/* 宸＄嚎妯″紡绾跨▼ */
void AUTO_MODE(void* paramenter)
{
    AutoModeInfo AMInfo;
    HandleModeInfo ClearBuf;

    //榛橈拷?锟芥寕璧疯嚜鍔ㄦā锟�?
    rt_thread_suspend(rt_thread_self());
    rt_schedule();
    while(1)
    {
        if(rt_mq_recv(AutoModemq,&AMInfo,sizeof(AutoModeInfo),RT_WAITING_FOREVER) == RT_EOK)
        {
            //锟�?鍔ㄦ帶鍒舵秷锟�?闃熷垪鎺ユ敹鍒版暟锟�?锛屽皢鍒囨崲鍒版墜鏌勬ā锟�?
            if(!rt_strcmp(AMInfo.ModeChange,"HANDLE START"))
            {
                //灏咹andleModemq闃熷垪锟�?鎵€鏈夊唴瀹规竻锟�?
                while(1)
                {
                    if(rt_mq_recv(HandleModemq,&ClearBuf,sizeof(HandleModeInfo),RT_WAITING_NO) != RT_EOK)
                        break;
                }
                Task_Thruster_AllStop();                //鎵€鏈夋帹杩涘櫒鍋滆浆

                printf("Switch to HANDLE Mode\r\n");
                rt_enter_critical();                    //璋冨害鍣ㄤ笂锟�?
                rt_thread_suspend(rt_thread_self());    //鎸傝捣锟�?绾跨▼
                rt_thread_resume(thread4);              //鎭�锟�?锟芥墜鍔ㄦ帶鍒剁嚎锟�?
                rt_exit_critical();                     //璋冨害鍣ㄨВ锟�?
                rt_schedule();                          //绔嬪嵆鎵э拷?锟戒竴娆¤皟锟�?
            }
            else
            {
                //锟�?鍔ㄦā寮忥拷?锟界悊鍑芥暟锛屾牴锟�?娑堟伅闃熷垪锟�?浼犳潵鐨勯粦绾匡拷?锟藉害鏀瑰彉鎺ㄨ繘鍣≒WM
                Task_AutoMode_Process(AMInfo);
            }
        }

        // printf("AUTO\r\n");
        // Drv_Delay_Ms(1000);
        Drv_Delay_Ms(1);    //璁╁嚭CPU璧勬簮缁欎綆浼樺厛绾х嚎锟�?
    }
}

/* 定深控制 */
void DepthControl(void* paramenter)
{
    DepthControlInfo DCInfo;
    float ExpDepth = 0.0f;
    float CurrDepth = 0.0f;

    while(1)
    {
        //定深数据消息队列接收到数据，将开始定深控制
        if(rt_mq_recv(DepthControlmq,&DCInfo,sizeof(DepthControlInfo),RT_WAITING_NO) == RT_EOK)
        {
            ExpDepth = DCInfo.setDepth;
            // printf("Exp:%0.2f\r\n",ExpDepth);
        }

//        printf("Exp:%0.2f\r\n",ExpDepth);
        //获取当前深度
        CurrDepth = MS5837.fDepth;
        //定深控制函数
        
            task_DepthControl_Process(CurrDepth,ExpDepth);

        Drv_Delay_Ms(600);    //每隔一段时间进行一次定深
    }
}


/*俯仰、机械爪及探照灯*/
void PlusControl(void* paramenter)
{
	PWMInfo.PWMout[claw_shouder_Speed] = 1000;
	PWMInfo.PWMout[claw_elbow_Speed] = Servo_Angle_To_HightTime(CLAW_CATCH_STOP_ANGLE);
    //PWMInfo.PWMout[claw_elbow_Speed] = Servo_Angle_To_HightTime(CLAW_SHOUDER_STOP_ANGLE);
	PWMInfo.PWMout[claw_catchSpeed] = 1250;
    PWMInfo.PWMout[light_level_1] = 1500;
	Mode_control |= 0x80; //Mode_control第八位为未初始化标志位
 //   State_control |= 0x10;
    while(1)                                                                    
    {		
		//Mode_control第一位为yaw自适应的控制位，置1为开启PID控制，0为关闭
		//Yaw的PID控制的初始化
		// if((Mode_control & 0x80) && (Mode_control & 0x01))
		// {
		// 	//Exp_AngleInfo.Yaw = JY901S.stcAngle.ConYaw;
		// 	concon_YAW = JY901S.stcAngle.ConYaw;
		// 	Mode_control &=~0x80;
		// }
		// else if((!(Mode_control & 0x80)) && (!(Mode_control & 0x01)))
		// {
		// 	Mode_control |=0x80;
		// }

		// if((State_control & 0x10) && (State_control & 0x01))
		// {
		// 	State_control &=~0x10;
		// }
		// else if((!(State_control & 0x10)) && (!(State_control & 0x01)))
		// {
		// 	State_control |=0x10;
		// }
        
        // //右摇杆左右控制转向
		// if(right_rocker==3)
		// 	{ 
		// 		Exp_AngleInfo.Yaw -= 0.5;
		// 	}
		// else if(right_rocker==4)
		// 	{ 
		// 		Exp_AngleInfo.Yaw += 0.5;
		// 	}
		//十字键左右控制俯仰
		// if(x_y_z_pitch & 0x01)
		// 	{ 
		// 		Exp_AngleInfo.Pitch += 0.5;
		// 		if(Exp_AngleInfo.Pitch > 150)
		// 			Exp_AngleInfo.Pitch = 150;
		// 	}
		// else if(x_y_z_pitch & 0x02)
		// 	{ 
		// 		Exp_AngleInfo.Pitch -= 0.5;
		// 		if(Exp_AngleInfo.Pitch < -150)
		// 			Exp_AngleInfo.Pitch = -150;
		// 	}
            
        //左右摇杆按键分别对应在外夹取物品状态与回框放置物品状态
        // if(State_control & 0x01)
        //     {
        //         DCInfo.setDepth += 0.1;
        //         if(DCInfo.setDepth > 100.0)
        //             DCInfo.setDepth = 100.0;
        //     }
        // else if(State_control & 0x02)
        //     {
        //         DCInfo.setDepth -= 0.1;
        //         if(DCInfo.setDepth < 0.0)
        //             DCInfo.setDepth = 0.0;
        //     }

		//左摇杆上下控制大臂
		if(left_rocker == 0)
        { ;
        }//锟�?
		
		else if(left_rocker == 2)
		{
			PWMInfo.PWMout[claw_shouder_Speed] += 5;
			if(PWMInfo.PWMout[claw_shouder_Speed] > 1170)
            PWMInfo.PWMout[claw_shouder_Speed] = 1170;
			if(PWMInfo.PWMout[claw_shouder_Speed] > 2500) 
            PWMInfo.PWMout[claw_shouder_Speed] = 2500;
			if(PWMInfo.PWMout[claw_shouder_Speed] < 500) 
            PWMInfo.PWMout[claw_shouder_Speed] = 500;
		}
		else if(left_rocker == 1)
		{
			PWMInfo.PWMout[claw_shouder_Speed] -= 5;
			if(PWMInfo.PWMout[claw_shouder_Speed] < 800 )
            PWMInfo.PWMout[claw_shouder_Speed] = 800;
			if(PWMInfo.PWMout[claw_shouder_Speed] > 2500) 
            PWMInfo.PWMout[claw_shouder_Speed] = 2500;
			if(PWMInfo.PWMout[claw_shouder_Speed] < 500) 
            PWMInfo.PWMout[claw_shouder_Speed] = 500;
		}
        else{ ;}

        //右摇杆上下控制小臂
        if(right_rocker == 0)
        { 
        //     printf("you");
        }
		else if(right_rocker == 1)
		{
            // printf("/naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/n");
			PWMInfo.PWMout[claw_elbow_Speed] -= CLAW_STEP+8;
			if(PWMInfo.PWMout[claw_elbow_Speed] > 2500) 
            PWMInfo.PWMout[claw_elbow_Speed] = 2500;
			if(PWMInfo.PWMout[claw_elbow_Speed] < 500) 
            PWMInfo.PWMout[claw_elbow_Speed] = 500;
		}
		else if(right_rocker == 2)
		{
			PWMInfo.PWMout[claw_elbow_Speed] += CLAW_STEP+8;
			if(PWMInfo.PWMout[claw_elbow_Speed] > 2500) 
            PWMInfo.PWMout[claw_elbow_Speed] = 2500;
			if(PWMInfo.PWMout[claw_elbow_Speed] < 500) 
            PWMInfo.PWMout[claw_elbow_Speed] = 500;
		}    
        //摄像头云台
        else if(right_rocker == 3)
        {
            PWMInfo.PWMout[light_level_1] += 15;
            if(PWMInfo.PWMout[light_level_1] > 1790) 
            PWMInfo.PWMout[light_level_1] = 1790;
            // right_rocker = 0;
        }
        else if(right_rocker == 4)
        {
            PWMInfo.PWMout[light_level_1] -= 15;
            if(PWMInfo.PWMout[light_level_1] < 1450) 
            PWMInfo.PWMout[light_level_1] = 1450;
            // right_rocker = 0;
        }
        else{ ;}

		//RB和LB控制夹和松
		if(Mode_control & 0x02)
		{
			PWMInfo.PWMout[claw_catchSpeed] -= CLAW_STEP+5;
			if(PWMInfo.PWMout[claw_catchSpeed] < 1190) 
            PWMInfo.PWMout[claw_catchSpeed] = 1190;

		}
		else if(Mode_control & 0x04)
		{
			PWMInfo.PWMout[claw_catchSpeed] += CLAW_STEP+5;
			if(PWMInfo.PWMout[claw_catchSpeed] > 1575) 
            PWMInfo.PWMout[claw_catchSpeed] = 1575;
			if(PWMInfo.PWMout[claw_catchSpeed] > 2500) 
            PWMInfo.PWMout[claw_catchSpeed] = 2500;
			if(PWMInfo.PWMout[claw_catchSpeed] < 500) 
            PWMInfo.PWMout[claw_catchSpeed] = 500;
		}
		else{ ;}
		
        // printf("%d\n",PWMInfo.PWMout[light_level_1]);
        //摄像头云台

        //调试用
        //printf("Y %d %d %d\r\n",PWMInfo.PWMout[claw_shouder_Speed],PWMInfo.PWMout[claw_elbow_Speed],PWMInfo.PWMout[claw_catchSpeed]);
		//printf("L %d\r\n",PWMInfo.PWMout[light_level_1]);
        //写入机械臂及灯光pwm值
        Task_Servo_AllStart(PWMInfo.PWMout);
		//延时
		Drv_Delay_Ms(100);
    }
}

/* 汇报PWMout值 */
void ReportPWMout(void* paramenter)
{
    while(1)
    {
//      printf("h: %d %d %d %d\r\n",
//			PWMInfo.PWMout[h_wheel1_speed],
//			PWMInfo.PWMout[h_wheel2_speed],
//			PWMInfo.PWMout[h_wheel3_speed],
//			PWMInfo.PWMout[h_wheel4_speed] );
			
			// printf("v: %d %d %d %d\r\n \r\n",
			// PWMInfo.PWMout[v_wheel1_speed],
			// PWMInfo.PWMout[v_wheel2_speed],
			// PWMInfo.PWMout[v_wheel3_speed],
			// PWMInfo.PWMout[v_wheel4_speed] );
        // printf("big:%d small:%d zhua:%d camera:%d\r\n",
        //         PWMInfo.PWMout[claw_shouder_Speed],
        //         PWMInfo.PWMout[claw_elbow_Speed],
        //         PWMInfo.PWMout[claw_catchSpeed],
        //         PWMInfo.PWMout[light_level_1]);
        Drv_Delay_Ms(500);    
    }
}

/* 娴嬭瘯绾跨▼ */
void TestThread(void* paramenter)
{
		Drv_GPIO_Reset(&demoGPIO[2]);
		Drv_Delay_Ms(1000);
		Drv_GPIO_Set(&demoGPIO[2]);
		Drv_Delay_Ms(1000);
}

