#include "task_conf.h"
#include "usercode.h"
#include "config.h"

extern PWMInfo_T PWMInfo;

/* 深度控制函数，需要当前深度和期望深度 */
void task_DepthControl_Process(float Curr,float Exp)
{
//    DepthPID_out = 0.0;         //PID计算后的结果

    DepthPID_out += -Algo_PID_Calculate(&DepthPID,Curr,Exp);
	
}

/*pitch轴抗旋*/
void task_pitch_data_handle(float Curr ,float Exp)  
{
	PitchPID_out = 0.0;

	if(Curr <= 0 && Curr >= -180)       //防止在水平角度时突变值
	{
		Curr += 360;
	}
    //测试jy901s
	// printf("PITCH %f\n\r",Curr_pitch);
	//计算并得到pitchPIDout的值
    PitchPID_out += -Algo_PID_Calculate(&PitchPID, Curr, Exp);
}
