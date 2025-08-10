#include "task_conf.h"
#include "usercode.h"
#include "config.h"

extern PWMInfo_T PWMInfo;

extern DepthControlInfo DCInfo;

void HandleMode_data_storage(char key,char Press);

//手动模式主函数
void Task_HandleMode_Process(HandleModeInfo HMInfo)
{

    char key = 0;         
    char Press = 0;           
	
    //存储手柄给出的按键信息和是否是按下
    key = HMInfo.fNum[0];
    Press = HMInfo.fNum[1];
	
	//信息储存
    HandleMode_data_storage(key,Press);
	
}

void HandleMode_data_storage(char key,char Press)
{
	DepthControlInfo DCInfo;
	printf("key = %d , press = %d\r\n",key,Press);
	switch (key)
	{
	//x_y_z_pitch
		//x
		case 2://左
			if(Press)
			{
				if(!(x_y_z_pitch & 128))
					x_y_z_pitch += 128;
			}
			else
			{
				if(x_y_z_pitch & 128)
					x_y_z_pitch -= 128;
			}
		break;
			
		case 1://右
			if(Press)
			{
				if(!(x_y_z_pitch & 64))
					x_y_z_pitch += 64;
			}
			else
			{
				if(x_y_z_pitch & 64)
					x_y_z_pitch -= 64;
			}
		break;
			
			//y
		case 3:
			if(Press)
			{
				if(!(x_y_z_pitch & 32))
					x_y_z_pitch += 32;
			}
			else
			{
				if(x_y_z_pitch & 32)
					x_y_z_pitch -= 32;
			}
		break;
			
		case 0:
			if(Press)
			{
				if(!(x_y_z_pitch & 16))
					x_y_z_pitch += 16;
			}
			else
			{
				if(x_y_z_pitch & 16)
					x_y_z_pitch -= 16;
			}
		break;
			
			//z
		case 7://下沉
			if(Press)
			{
				if(!(x_y_z_pitch & 8))
					x_y_z_pitch += 8;
			}
			else
			{
				if(x_y_z_pitch & 8)
					x_y_z_pitch -= 8;
				DCInfo.setDepth = MS5837.fDepth;
				rt_mq_send(DepthControlmq,&DCInfo,sizeof(DepthControlInfo));
			}
		break;
			
		case 4:
			if(Press)
			{
				if(!(x_y_z_pitch & 4))
					x_y_z_pitch += 4;
			}
			else
			{
				if(x_y_z_pitch & 4)
					x_y_z_pitch -= 4;
				DCInfo.setDepth = MS5837.fDepth;
				rt_mq_send(DepthControlmq,&DCInfo,sizeof(DepthControlInfo));
			}
		break;
			
			//pitch俯仰
		case 5:
			if(Press)
			{
				if(!(x_y_z_pitch & 2))
					x_y_z_pitch += 2;
			}
			else
			{
				if(x_y_z_pitch & 2)
					x_y_z_pitch -= 2;
			}
		break;
			
		case 6:
			if(Press)
			{
				if(!(x_y_z_pitch & 1))
					x_y_z_pitch += 1;
			}
			else
			{
				if(x_y_z_pitch & 1)
					x_y_z_pitch -= 1;
			}
		break;
			
			
			//夹、松控制使用RB和LB
			
		   case 10:
			if(Press)
				{
					if(!(Mode_control & 2))
						Mode_control += 2;
				}
			else
			{
					if((Mode_control & 2))
						Mode_control -= 2;
			}
	    	break;

			case 11:
			if(Press)
				{
					if(!(Mode_control & 4))
						Mode_control += 4;
				}
			else
			{
					if((Mode_control & 4))
						Mode_control -= 4;
			}
	    	break;
			
			case 18:
			left_rocker = 1;
		break;
			
			case 19:
			left_rocker = 2;
		break;
			
			case 20://右
			left_rocker = 3;
		break;
			
			case 21://左
			left_rocker = 4;
		break;
			
			case 23:
				left_rocker = 0;
	       break;

			//右左中
			case 14:
				right_rocker = 1;
				//Mode_control |= 1;
			break;
			
			case 15:
				right_rocker = 2;
			//	Mode_control |= 1;
			break;
			
			case 16://右
			    right_rocker = 3;
				//Mode_control |= 1;
			break;
			
			case 17://左
			    right_rocker = 4;
				//Mode_control |= 1;
			break;
			
			case 22:
				right_rocker = 0;
				//Mode_control |= 1;
			break;
			
		   case 12:
			if(Press)
				{
					if(!(State_control & 2))
						State_control += 2;
				}
				else
				{
						//清0状态位，进入停止
						if((State_control & 2))
						State_control -= 2;
				}
	    	break;
		   case 13:
			if(Press)
				{
					if(!(State_control & 1))
						State_control += 1;
				}
				else
				{
						//清0状态位，进入停止
						if((State_control & 1))
						State_control -= 1;
				}
	    	break;
		//灯光控制按键
		// //亮灯
		// case 8:
		// 	if(Press)
		// 	{
		// 		if(!(Light_control & 1))
		// 			Light_control += 1;
		// 	}
		// break;
		// //暗灯
		// case 9:
		// 	if(Press)
		// 	{
		// 		if(!(Light_control & 2))
		// 			Light_control += 2;
		// 	}
		// break;

		//start键
			//定深总开关使用DepthFlag第一位
			case 8:
			if(Press)
				{//计数器
					if(((DepthFlag >> 1 ) % 2) == 0)
						DepthFlag |= 2;
					else
						DepthFlag -= 2;
					DCInfo.setDepth = MS5837.fDepth;
					rt_mq_send(DepthControlmq,&DCInfo,sizeof(DepthControlInfo));
				}
			else
				{
					// Expect_angle_Init();
					// concon_YAW = JY901S.stcAngle.ConYaw;
					if(((DepthFlag >> 1 ) % 2) == 1)
						DepthFlag |= 1;
					else
						DepthFlag &= ~1;
				}
			break;
		//back键
		//慢速状态切换
		case 9:
			if(Press)
				{
					//奇偶计次
					if(((SpeedMode >> 1 ) % 2) == 0)
						SpeedMode |= 2;
					else
						SpeedMode -= 2;
				}
			else
				{
					if(((SpeedMode >> 1 ) % 2) == 1)
						SpeedMode |= 1;
					else
						SpeedMode &= ~1;
				}
			break;
    }
}
