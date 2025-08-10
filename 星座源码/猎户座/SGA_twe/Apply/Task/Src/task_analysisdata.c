#include "task_conf.h"
#include "usercode.h"
#include "config.h"

static char AnalysisData[5][15];
static int AnalysisNum = 0;

/* ????????��??????????????? */
void Task_AnalysisData(uint8_t *DataBuf)
{
    //????????0
    rt_memset(AnalysisData,0,sizeof(AnalysisData));
    HandleModeInfo HMInfo = {" ",0,0,0};
    AutoModeInfo AMInfo = {" ",0};
    DepthControlInfo DCInfo = {0};
    AnalysisNum = 0;

    //???????��???????????????????
    AnalysisNum = sscanf((char *)DataBuf,"%s %s %s %s %s",
                        AnalysisData[0],
                        AnalysisData[1],
                        AnalysisData[2],
                        AnalysisData[3],
                        AnalysisData[4]);
      printf("%d ,%s,%s,%s,%s,%s\r\n",AnalysisNum,AnalysisData[0],AnalysisData[1],AnalysisData[2],AnalysisData[3],AnalysisData[4]);

    if(!AnalysisNum) return;
    /* ????????????? */

    //????
    else if(!rt_strcmp(AnalysisData[0],"JSB"))
    {
        HMInfo.fNum[0] = strtod(AnalysisData[1],NULL);

        if(!rt_strcmp(AnalysisData[2],"Press"))
        {
            HMInfo.fNum[1] = true;//??1????Press
        }
        else
        {
            HMInfo.fNum[1] = false; //??0????Release
        }
		
    //???????????????????????????????????
    rt_mq_send(HandleModemq,&HMInfo,sizeof(HandleModeInfo));
	}
	
    //??????
    else if(!rt_strcmp(AnalysisData[0],"JSV1"))
    {
		short angle = strtof(AnalysisData[1],NULL);
		
		//?????????
		if(angle == 0)
		    HMInfo.fNum[0] = 18;
		else if(angle == 180)
		    HMInfo.fNum[0] = 19;
		else if(angle == 90)
		    HMInfo.fNum[0] = 20;
		else if(angle == -90)
		    HMInfo.fNum[0] = 21;
		else if(angle ==999)
		    HMInfo.fNum[0] = 23;
        
    //???????????????????????????????????
    rt_mq_send(HandleModemq,&HMInfo,sizeof(HandleModeInfo));
    }
	
    //??????
	    else if(!rt_strcmp(AnalysisData[0],"JSV2"))
    {
		short angle = strtof(AnalysisData[1],NULL);
		
		//?????????
		if(angle == 0)
		    HMInfo.fNum[0] = 14;
		else if(angle == 180)
		    HMInfo.fNum[0] = 15;
		else if(angle == 90)
		    HMInfo.fNum[0] = 16;
		else if(angle == -90)
		    HMInfo.fNum[0] = 17;
		else if(angle ==999)
		    HMInfo.fNum[0] = 22;
        
    //???????????????????????????????????
    rt_mq_send(HandleModemq,&HMInfo,sizeof(HandleModeInfo));
    }
	
//???
	    else if(!rt_strcmp(AnalysisData[0],"BJ"))
    {
		speed_kH = strtof(AnalysisData[1],NULL);
		speed_kV = strtof(AnalysisData[2],NULL);
		
    /*???????????????????????????????????
    rt_mq_send(HandleModemq,&HMInfo,sizeof(HandleModeInfo));*/
    }
    


    //???��?????
    /*else if(!rt_strcmp(AnalysisData[0],"MODE"))
    {
        //printf("%s\r\n",AnalysisData[1]);
        if(!rt_strcmp(AnalysisData[1],"AUTO"))
        {
            //???????????????????????????
            rt_memcpy(HMInfo.ModeChange,"AUTO START",sizeof("AUTO START"));
            rt_mq_send(HandleModemq,&HMInfo,sizeof(HandleModeInfo));
        }
        else if(!rt_strcmp(AnalysisData[1],"HANDLE"))
        {
            //???????????????????????
            rt_memcpy(AMInfo.ModeChange,"HANDLE START",sizeof("HANDLE START"));
            rt_mq_send(AutoModemq,&AMInfo,sizeof(AutoModeInfo));              
        }
    }*/

    //PID?
    else if(!rt_strcmp(AnalysisData[0],"PID"))
    {
        //????PID
        if(!rt_strcmp(AnalysisData[1],"DepthPID"))
        {
            HMInfo.fNum[0] = strtof(AnalysisData[2],NULL);
            HMInfo.fNum[1] = strtof(AnalysisData[3],NULL);
            HMInfo.fNum[2] = strtof(AnalysisData[4],NULL);

            //????????PID
            Algo_PID_Update(&DepthPID,HMInfo.fNum);
            printf("DepthPID %.2f %.2f %.2f\r\n",DepthPID.fKp,DepthPID.fKi,DepthPID.fKd);
        }
        //????PID
        else if(!rt_strcmp(AnalysisData[1],"YawPID"))
        {
            HMInfo.fNum[0] = strtof(AnalysisData[2],NULL);
            HMInfo.fNum[1] = strtof(AnalysisData[3],NULL);
            HMInfo.fNum[2] = strtof(AnalysisData[4],NULL);

            //????????PID
            Algo_PID_Update(&YawPID,HMInfo.fNum);
            printf("YawPID %.2f %.2f %.2f\r\n",YawPID.fKp,YawPID.fKi,YawPID.fKd);
        }
        //????PID
        else if(!rt_strcmp(AnalysisData[1],"AngleLoopPID"))
        {
            HMInfo.fNum[0] = strtof(AnalysisData[2],NULL);
            HMInfo.fNum[1] = strtof(AnalysisData[3],NULL);
            HMInfo.fNum[2] = strtof(AnalysisData[4],NULL);

            //??????????PID
            Algo_PID_Update(&AngleLoopPID,HMInfo.fNum);
            printf("AngleLoopPID %.2f %.2f %.2f\r\n",AngleLoopPID.fKp,AngleLoopPID.fKi,AngleLoopPID.fKd);
        }
        //��???PID
        else if(!rt_strcmp(AnalysisData[1],"PositionLoopPID"))
        {
            HMInfo.fNum[0] = strtof(AnalysisData[2],NULL);
            HMInfo.fNum[1] = strtof(AnalysisData[3],NULL);
            HMInfo.fNum[2] = strtof(AnalysisData[4],NULL);

            //????��???PID
            Algo_PID_Update(&PositionLoopPID,HMInfo.fNum);
            printf("PositionLoopPID %.2f %.2f %.2f\r\n",PositionLoopPID.fKp,PositionLoopPID.fKi,PositionLoopPID.fKd);
        }
        //????PID
        // else if(!rt_strcmp(AnalysisData[1],"LinePatrolPID"))
        // {
        //     HMInfo.fNum[0] = strtof(AnalysisData[2],NULL);
        //     HMInfo.fNum[1] = strtof(AnalysisData[3],NULL);
        //     HMInfo.fNum[2] = strtof(AnalysisData[4],NULL);
            
        //     //????????PID
        //     Algo_PID_Update(&LinePatrolPID,HMInfo.fNum);
        //     printf("LinePatrolPID %.2f %.2f %.2f\r\n",LinePatrolPID.fKp,LinePatrolPID.fKi,LinePatrolPID.fKd);
        // }
    }
}

