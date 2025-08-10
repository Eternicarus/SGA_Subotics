#include "task_conf.h"
#include "usercode.h"
#include "config.h"

extern PWMInfo_T PWMInfo;

extern DepthControlInfo DCInfo;

void Task_HandleMode_Clear(void)
{
		DepthPID_out = 0;
		x_y_z_pitch = 0;
		right_rocker = 0;
		left_rocker = 0;
		Mode_control = 0;
}

void Task_HandleMode_Process(HandleModeInfo HMInfo)
{
	DepthControlInfo DCInfo;
	if(!rt_strcmp(HMInfo.Change, "MS"))         //左摇杆--平移
	{
		switch((int)HMInfo.fNum[0])
		{
			case 0:
			                                //右
				if((int)HMInfo.fNum[1])
					x_y_z_pitch |= 0x40;
				else
					x_y_z_pitch &= ~0x40;
				break;
			
			case 1:
			                                //前
				if((int)HMInfo.fNum[1])
					x_y_z_pitch |= 0x20;
				else
					x_y_z_pitch &= ~0x20;
				break;
			
			case 2:
			                                //左
				if((int)HMInfo.fNum[1])
					x_y_z_pitch |= 0x80;
				else
					x_y_z_pitch &= ~0x80;
				break;
			
			case 3:
			                                //后
				if((int)HMInfo.fNum[1])
					x_y_z_pitch |= 0x10;
				else
					x_y_z_pitch &= ~0x10;
				break;
			
			case 9:        //识别位为松手的0，开启沉底下潜模式
				if(!(int)HMInfo.fNum[1])
				{
					if(((sinkflag >> 1) & 0x01) == 0)//如果第一位是0
					{
						sinkflag |= 0x02;
						
					    break;
					}//设置为1
				else
				{
					sinkflag &= ~0x02;//清除
					break;
				}
				}
				
		}			
	}
	else if(!rt_strcmp(HMInfo.Change, "MP"))         //右摇杆--左右转向--上下深度
	{
		switch((int)HMInfo.fNum[0])
		{
			case 0:
			                            //右
				if((int)HMInfo.fNum[1])
					left_rocker = 3; 
				else
					left_rocker = 0; 
				break;
			
			case 1:
			    
				if((int)HMInfo.fNum[1])
				{
					x_y_z_pitch |= 0x08;
					break;
				}
				else
				{
					x_y_z_pitch &= ~0x08;
					DCInfo.setDepth = MS5837.fDepth;
					rt_mq_send(DepthControlmq,&DCInfo,sizeof(DepthControlInfo));
					break;
				}
			
			case 2:
			                              //左
				if((int)HMInfo.fNum[1])
					left_rocker = 4;
				else
					left_rocker = 0;
				break;
			
			case 3:
			         
				if((int)HMInfo.fNum[1])
				{
					x_y_z_pitch |= 0x04;
					break;
				}
				else
				{
					x_y_z_pitch &= ~0x04;
					DCInfo.setDepth = MS5837.fDepth;
					rt_mq_send(DepthControlmq,&DCInfo,sizeof(DepthControlInfo));
					break;
				}
		    case 9:            //识别位为末位0，开启顶部保持状态
				if(!(int)HMInfo.fNum[1])
				{
					if(((floatflag >> 1) & 0x01) == 0)//如果第一位是0
					{
						floatflag |= 0x02;
						break;
					
					}//设置为1
				    else
				   {
					    floatflag &= ~0x02;//清除
					   break;
				   }
				}
				
			
		}
	}
	else if(!rt_strcmp(HMInfo.Change, "MT"))       //扳机
	{
		switch((int)HMInfo.fNum[0])
		{
			case 0:
				Mode_control = 9;
				break;
			case 1:
				Mode_control = 10;
				break;
		}
	}
	else if(!rt_strcmp(HMInfo.Change, "MR"))			//字母键
	{
		switch((int)HMInfo.fNum[0])     
		{
			case 0:									//针筒抽
				Mode_control = 3;
				break;
			case 1:									//下机械臂上
				Mode_control = 2;
				break;
			case 2:									//针筒压
				Mode_control = 1;
				break;
			case 3:									//下机械臂下
                Mode_control = 4;
				break;
		}
	}
	else if(!rt_strcmp(HMInfo.Change, "MM"))			//十字键 RB LB键
	{
		switch((int)HMInfo.fNum[0])
		{
			case 0:								//针筒抽
				Mode_control = 11;
				break;
			case 1:								//摄像机云台下
				Mode_control = 6;
				break;
			case 2:								//针筒推
				Mode_control = 12;
				break;
			case 3:								//摄像机云台上
				Mode_control = 5;
				break;
			case 5:
				Mode_control = 7;				//上机械爪
				break;
			case 7:
				Mode_control = 8;				//上机械爪
				break;
		}
	}
	else if(!rt_strcmp(HMInfo.Change, "MD"))			//start back键
	{
		switch((int)HMInfo.fNum[0])
		{
			case 1:								//定深开关键
				//计数器
				if(((DepthFlag >> 1) & 0x01) == 0)//如果第一位是0
					{
						DepthFlag |= 0x02;
					DCInfo.setDepth = MS5837.fDepth;
					rt_mq_send(DepthControlmq,&DCInfo,sizeof(DepthControlInfo));
				    printf("Depth succeeded!\n\r");
					}//设置为1
				else
				{
					DepthFlag &= ~0x02;//清除
					DepthPID_out = 0.0;
					printf("Depth disconnected!\n\r");
				}
				break;
			case 0:
                  pHflag = 1;
				break;
		}
	}
	
	
}
