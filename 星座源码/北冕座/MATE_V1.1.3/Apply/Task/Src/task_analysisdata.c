#include "task_conf.h"
#include "usercode.h"
#include "config.h"

static char AnalysisData[5][15];
static int AnalysisNum = 0;

/* 分析从上位机处接收到的数据 */
void Task_AnalysisData(uint8_t *DataBuf)
{
    // 将变量清0
    rt_memset(AnalysisData, 0, sizeof(AnalysisData));
    HandleModeInfo HMInfo = {" ", " ", 0, 0, 0, " "};
    AutoModeInfo AMInfo = {" ", 0};
    DepthControlInfo DCInfo = {0};
    AnalysisNum = 0;

    // 根据格式切分字符串存放到二维数组中
    AnalysisNum = sscanf((char *)DataBuf, "%s %s %s %s %s",
                         AnalysisData[0],
                         AnalysisData[1],
                         AnalysisData[2],
                         AnalysisData[3],
                         AnalysisData[4]);
//     printf("%d ,%s,%s,%s,%s,%s\r\n",AnalysisNum,AnalysisData[0],AnalysisData[1],AnalysisData[2],AnalysisData[3],AnalysisData[4]);

    if (!AnalysisNum)
        return;
    /* 根据帧头选择任务 */

    // 手柄摇杆与扳机值  "@ 标识 数值 （是否按下）$"  
    else if (!rt_strcmp(AnalysisData[0], "@"))
    {
			// 更新 HMInfo 的内容
        if (AnalysisNum >= 3) {
            strcpy(HMInfo.Change, AnalysisData[1]); 
            HMInfo.fNum[0] = strtof(AnalysisData[2],NULL);
			HMInfo.fNum[1] = strtof(AnalysisData[3],NULL);
        }
				
//		printf("%s %f %f\r\n",HMInfo.Change, HMInfo.fNum[0],HMInfo.fNum[1]);

        // 将数据发送给手柄控制线程
        rt_mq_send(HandleModemq, &HMInfo, sizeof(HandleModeInfo));
    }
	
//	else if (!rt_strcmp(AnalysisData[0], "p"))
//	{
//		pHflag = 1;
//	}

    // 手柄按键状态   1："@按键标识$"   2模式按键（8,9）：“@按键标识 0/1”
//    else if (AnalysisData[0][0] == '@')
//    {
//				if(!rt_strcmp(AnalysisData[0], "@MRU0$")){
//					HMInfo.fNum[2] = 0;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MRB0$")){
//					HMInfo.fNum[2] = 1;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MRD0$")){
//					HMInfo.fNum[2] = 2;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MRF0$")){
//					HMInfo.fNum[2] = 3;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MMB0$")){
//					HMInfo.fNum[2] = 4;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MRR0$")){
//					HMInfo.fNum[2] = 5;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MRL0$")){
//					HMInfo.fNum[2] = 6;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MMF0$")){
//					HMInfo.fNum[2] = 7;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MDM0")){
//					HMInfo.fNum[2] = 8;
////					HMInfo.fNum[2] = strtof(AnalysisData[1],NULL);
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MDP0")){
//					HMInfo.fNum[2] = 9;
////					HMInfo.fNum[2] = strtof(AnalysisData[1],NULL);
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MMD0$")){
//					HMInfo.fNum[2] = 10;
//				}
//					
//				else if(!rt_strcmp(AnalysisData[0], "@MMU0$")){
//					HMInfo.fNum[2] = 11;
//				}
//					
//				
//				strcpy(HMInfo.Change, AnalysisData[0]);
//        // 将数据发送给手柄控制线程
//			
////		printf("%s %f\r\n",HMInfo.Change, HMInfo.fNum[1]);
//			
//        rt_mq_send(HandleModemq, &HMInfo, sizeof(HandleModeInfo));
//    }

    // 自动巡线模式角度值
    else if (!rt_strcmp(AnalysisData[0], "LP"))
    {
        // 保存数据，LP 角度 位置偏移
        AMInfo.BlackAngle = strtof(AnalysisData[1], NULL);
        AMInfo.CenterShift = strtof(AnalysisData[2], NULL);
        rt_mq_urgent(AutoModemq, &AMInfo, sizeof(AutoModeInfo));
    }

    // 定深数值
    else if (!rt_strcmp(AnalysisData[0], "D"))
    {
        DCInfo.setDepth = strtof(AnalysisData[1], NULL);
        rt_mq_send(DepthControlmq, &DCInfo, sizeof(DepthControlInfo));
    }

    // 模式切换命令
    else if (!rt_strcmp(AnalysisData[0], "MODE"))
    {
        // printf("%s\r\n",AnalysisData[1]);
        if (!rt_strcmp(AnalysisData[1], "AUTO"))
        {
            // 挂起手柄控制模式，启动自动巡线模式
            rt_memcpy(HMInfo.ModeChange, "AUTO START", sizeof("AUTO START"));
            rt_mq_send(HandleModemq, &HMInfo, sizeof(HandleModeInfo));
        }
        else if (!rt_strcmp(AnalysisData[1], "HANDLE"))
        {
            // 挂起自动巡线模式，启动手柄模式
            rt_memcpy(AMInfo.ModeChange, "HANDLE START", sizeof("HANDLE START"));
            rt_mq_send(AutoModemq, &AMInfo, sizeof(AutoModeInfo));
        }
    }

    // PID值
    else if (!rt_strcmp(AnalysisData[0], "PID"))
    {
        // 深度环PID
        if (!rt_strcmp(AnalysisData[1], "DepthPID"))
        {
            HMInfo.fNum[0] = strtof(AnalysisData[2], NULL);
            HMInfo.fNum[1] = strtof(AnalysisData[3], NULL);
            HMInfo.fNum[2] = strtof(AnalysisData[4], NULL);

            // 更新深度环PID
            Algo_PID_Update(&DepthPID, HMInfo.fNum);
            printf("DepthPID %.2f %.2f %.2f\r\n", DepthPID.fKp, DepthPID.fKi, DepthPID.fKd);
        }
        // 艏向环PID
        else if (!rt_strcmp(AnalysisData[1], "YawPID"))
        {
            HMInfo.fNum[0] = strtof(AnalysisData[2], NULL);
            HMInfo.fNum[1] = strtof(AnalysisData[3], NULL);
            HMInfo.fNum[2] = strtof(AnalysisData[4], NULL);

            // 更新艏向环PID
            Algo_PID_Update(&YawPID, HMInfo.fNum);
            printf("YawPID %.2f %.2f %.2f\r\n", YawPID.fKp, YawPID.fKi, YawPID.fKd);
        }
        // 角度环PID
        else if (!rt_strcmp(AnalysisData[1], "AngleLoopPID"))
        {
            HMInfo.fNum[0] = strtof(AnalysisData[2], NULL);
            HMInfo.fNum[1] = strtof(AnalysisData[3], NULL);
            HMInfo.fNum[2] = strtof(AnalysisData[4], NULL);

            // 更新艏角度环PID
            Algo_PID_Update(&AngleLoopPID, HMInfo.fNum);
            printf("AngleLoopPID %.2f %.2f %.2f\r\n", AngleLoopPID.fKp, AngleLoopPID.fKi, AngleLoopPID.fKd);
        }
        // 位置环PID
        else if (!rt_strcmp(AnalysisData[1], "PositionLoopPID"))
        {
            HMInfo.fNum[0] = strtof(AnalysisData[2], NULL);
            HMInfo.fNum[1] = strtof(AnalysisData[3], NULL);
            HMInfo.fNum[2] = strtof(AnalysisData[4], NULL);

            // 更新位置环PID
            Algo_PID_Update(&PositionLoopPID, HMInfo.fNum);
            printf("PositionLoopPID %.2f %.2f %.2f\r\n", PositionLoopPID.fKp, PositionLoopPID.fKi, PositionLoopPID.fKd);
        }
        // 巡线环PID
        //  else if(!rt_strcmp(AnalysisData[1],"LinePatrolPID"))
        //  {
        //      HMInfo.fNum[0] = strtof(AnalysisData[2],NULL);
        //      HMInfo.fNum[1] = strtof(AnalysisData[3],NULL);
        //      HMInfo.fNum[2] = strtof(AnalysisData[4],NULL);

        //     //更新巡线环PID
        //     Algo_PID_Update(&LinePatrolPID,HMInfo.fNum);
        //     printf("LinePatrolPID %.2f %.2f %.2f\r\n",LinePatrolPID.fKp,LinePatrolPID.fKi,LinePatrolPID.fKd);
        // }
    }
}


// int ParseMultipleFrames(const uint8_t *DataBuf)
// {
//     const char *p = (const char *)DataBuf;
//     const char *start, *end;
//     char tempBuf[MAX_FRAME_BUF];
//     int frameCount = 0;

//     while ((start = strchr(p, '@')) && (end = strchr(start, '$')) && frameCount < MAX_FRAMES)
//     {
//         int frameLen = end - start - 1;

//         // 边界保护
//         if (frameLen <= 0 || frameLen >= MAX_FRAME_BUF) {
//             p = end + 1;
//             continue;
//         }

//         // 拷贝去除帧头帧尾的中间数据
//         memcpy(tempBuf, start + 1, frameLen);
//         tempBuf[frameLen] = '\0';

//         // 使用 sscanf 拆分字段
//         sscanf(tempBuf, "%s %s %s %s %s",
//                FrameData[frameCount][0],
//                FrameData[frameCount][1],
//                FrameData[frameCount][2],
//                FrameData[frameCount][3],
//                FrameData[frameCount][4]);

//         frameCount++;
//         p = end + 1; // 继续处理后续帧
//     }

//     return frameCount;
// }