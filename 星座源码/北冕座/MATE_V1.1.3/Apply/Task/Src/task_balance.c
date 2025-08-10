#include "task_conf.h"
#include "usercode.h"
#include "config.h"

extern PWMInfo_T PWMInfo;
/* 使得左右机翼保持平衡 */
void Task_Balance_Process(void)
{
	float rollOutput = 0,pitchOutput = 0;
	// float Curr_roll = JY901S.stcAngle.ConPitch;
	float Curr_pitch = JY901S.stcAngle.ConRoll;
	// float Curr_yaw = concon_YAW;
	
	//计算每个维度的PID输出
    // rollOutput = Algo_PID_Calculate(&RollPID, Curr_roll, Exp_AngleInfo.Roll);
    pitchOutput = Algo_PID_Calculate(&PitchPID, Curr_pitch, Exp_AngleInfo.Pitch);
	
    // if(pitchOutput > 50)  pitchOutput = 50;
    // if(pitchOutput < -50)  pitchOutput = -50;
    if(x_y_z_pitch & 0x08)
    {
        if(pitchOutput > 30) pitchOutput = 30;
        if(pitchOutput < -30) pitchOutput = -30;
    }
	if(x_y_z_pitch & 0x04)
	{
        if(pitchOutput > 30) pitchOutput = 30;
        if(pitchOutput < -30) pitchOutput = -30;
    }

	PWMInfo.PWMout[h_wheel1_speed] += + pitchOutput; 
    PWMInfo.PWMout[h_wheel2_speed] += + pitchOutput; 
    PWMInfo.PWMout[h_wheel3_speed] += - pitchOutput; 
    PWMInfo.PWMout[h_wheel4_speed] += - pitchOutput;

    if(PWMInfo.PWMout[h_wheel1_speed] < 1350)  PWMInfo.PWMout[h_wheel1_speed] = 1350;
    if(PWMInfo.PWMout[h_wheel1_speed] > 1660)  PWMInfo.PWMout[h_wheel1_speed] = 1660;
	
    if(PWMInfo.PWMout[h_wheel2_speed] < 1350)  PWMInfo.PWMout[h_wheel2_speed] = 1350;
    if(PWMInfo.PWMout[h_wheel2_speed] > 1660)  PWMInfo.PWMout[h_wheel2_speed] = 1660;
	
    if(PWMInfo.PWMout[h_wheel3_speed] < 1350)  PWMInfo.PWMout[h_wheel3_speed] = 1350;
    if(PWMInfo.PWMout[h_wheel3_speed] > 1660)  PWMInfo.PWMout[h_wheel3_speed] = 1660;
	
    if(PWMInfo.PWMout[h_wheel4_speed] < 1350)  PWMInfo.PWMout[h_wheel4_speed] = 1350;
    if(PWMInfo.PWMout[h_wheel4_speed] > 1660)  PWMInfo.PWMout[h_wheel4_speed] = 1660;
}

void Expect_angle_Init(void)
{
	Exp_AngleInfo .Pitch = JY901S.stcAngle.ConRoll;
	Exp_AngleInfo .Roll = JY901S.stcAngle.ConPitch;
	Exp_AngleInfo .Yaw = JY901S.stcAngle.ConYaw;
}

