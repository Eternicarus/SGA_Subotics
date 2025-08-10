#include "task_conf.h"
#include "usercode.h"
#include "config.h"

extern PWMInfo_T PWMInfo;
extern DepthControlInfo DCInfo;

/* 运动控制主函数 */
void Task_Motion_Process(void)
{
	
	//将抽屉数组计算出来，存储PWMInfo，设定PWMInfo宏观值
	HandleMode_data_handle();
	// if(BalanceFlag)
	// 	//根据期望角度对PWMInfo进行修改,使得两个维度的旋�?保持平衡，俯仰�?�达到�?�期
	// 	Balance_data_handle();
	
	//限制最大输�?
	PWNOutput_limit();
	
	//将反�?的PWM输出增大�?倍数
	Thruster_nagative_data_handle(1.2);
	
	//设置�?�?推进器PWN输出
    Task_Thruster_pin_AllStart(PWMInfo.PWMout);
}

void HandleMode_data_handle()
{
	//各方向速度分量
	short x_Speed = 0;
	short y_Speed = 0;
	short YAWspeed = 0;
	// short z_Speed = 0;
	
	//根据储存按键数据和扳机速度系数处理出各方向速度分量的�?
	//按键
	//慢速（仅前进后退）
	if((x_y_z_pitch & 0x80) && (SpeedMode & 0x01))
		x_Speed += 40;
	if((x_y_z_pitch & 0x40) && (SpeedMode & 0x01))
	    x_Speed -= 40;
	if((x_y_z_pitch & 0x20) && (SpeedMode & 0x01))
		y_Speed += 40;
	if((x_y_z_pitch & 0x10) && (SpeedMode & 0x01))
	    y_Speed -= 40;
	if((left_rocker==3) && (SpeedMode & 0x01))
		YAWspeed = 40;
	if((left_rocker==4) && (SpeedMode & 0x01))
		YAWspeed = -40;
	//原速
	if((x_y_z_pitch & 0x80) && (!(SpeedMode & 0x01)))
		x_Speed += speed_kH * MAX_HPOWER  / 100;
	if((x_y_z_pitch & 0x40) && (!(SpeedMode & 0x01)))
	    x_Speed -= speed_kH * MAX_HPOWER / 100;
	if((x_y_z_pitch & 0x20) && (!(SpeedMode & 0x01)))
		y_Speed += speed_kH * MAX_HPOWER / 100;
	if((x_y_z_pitch & 0x10) && (!(SpeedMode & 0x01)))
	    y_Speed -= speed_kH * MAX_HPOWER / 100;
	if((left_rocker==3) && (!(SpeedMode & 0x01)))
		YAWspeed = YAWSPEED;
	if((left_rocker==4) && (!(SpeedMode & 0x01)))
		YAWspeed = -YAWSPEED-5;
 
	//计算推进器�?
	PWMInfo.PWMout[v_wheel1_speed] =  y_Speed -  x_Speed - YAWspeed + STOP_PWM_VALUE;
    PWMInfo.PWMout[v_wheel2_speed] =  y_Speed +  x_Speed - YAWspeed + STOP_PWM_VALUE;
	PWMInfo.PWMout[v_wheel3_speed] =  -(y_Speed +  x_Speed + YAWspeed) + STOP_PWM_VALUE ;//左后反转，原因不明，暂定推进器原因
   // PWMInfo.PWMout[v_wheel3_speed] =  -y_Speed -  x_Speed/abs(x_Speed)*(abs(x_Speed)-10) - YAWspeed + STOP_PWM_VALUE;
    PWMInfo.PWMout[v_wheel4_speed] =  y_Speed -  x_Speed + YAWspeed + STOP_PWM_VALUE;



	// uint16_t Hvalue;
	//�?改直线b�?
	/*if(z_Speed<2 && z_Speed>-2)
		Hvalue= z_Speed + STOP_PWM_VALUE;
	else if(z_Speed>2)
		Hvalue= z_Speed + STOP_PWM_VALUE;
	else if(z_Speed<-2)
		Hvalue= z_Speed + STOP_PWM_VALUE;*/
	// Hvalue= z_Speed + STOP_PWM_VALUE;
	
    // PWMInfo.PWMout[h_wheel1_speed] = z_Speed + STOP_PWM_VALUE;
    // PWMInfo.PWMout[h_wheel2_speed] = z_Speed + STOP_PWM_VALUE;
    // PWMInfo.PWMout[h_wheel3_speed] = z_Speed + STOP_PWM_VALUE;
    // PWMInfo.PWMout[h_wheel4_speed] = z_Speed + STOP_PWM_VALUE;
}

void Balance_data_handle(void)
{
// 	float rollOutput = 0,pitchOutput = 0;
// 	float Curr_roll = JY901S.stcAngle.ConPitch;
// 	float Curr_pitch = JY901S.stcAngle.ConRoll;
// 	// float Curr_yaw = concon_YAW;
	
// 	//计算每个维度的PID输出
//     rollOutput = Algo_PID_Calculate(&RollPID, Curr_roll, Exp_AngleInfo.Roll);
//     pitchOutput = Algo_PID_Calculate(&PitchPID, Curr_pitch, Exp_AngleInfo.Pitch);
// // 	if((!(Mode_control & 0x80))&&(Mode_control & 0x01))
// // 	{	
// // 		yawOutput = Algo_PID_Calculate(&YawPID, Curr_yaw, Exp_AngleInfo.Yaw);
// // //		printf("Y %f %f\r\n",yawOutput,Exp_AngleInfo.Yaw);
// // 	}
	
// 	//根据PIDout值�?�PWMInfo进�?�修�?
//     // PWMInfo.PWMout[v_wheel1_speed] -= yawOutput;
//     // PWMInfo.PWMout[v_wheel2_speed] += yawOutput;
//     // PWMInfo.PWMout[v_wheel3_speed] += yawOutput;
//     // PWMInfo.PWMout[v_wheel4_speed] -= yawOutput;
	
// 	PWMInfo.PWMout[h_wheel1_speed] += - pitchOutput + rollOutput; 
//     PWMInfo.PWMout[h_wheel2_speed] += - pitchOutput - rollOutput; 
//     PWMInfo.PWMout[h_wheel3_speed] += + pitchOutput + rollOutput; 
//     PWMInfo.PWMout[h_wheel4_speed] += + pitchOutput - rollOutput; 
}

void Thruster_nagative_data_handle(float k)
{
	int i = 0;
	for(;i<8;i++)
	{
		if(PWMInfo.PWMout[i]<1500)
			PWMInfo.PWMout[i] = 1500-(1500-PWMInfo.PWMout[i])*k;
	}
}

void PWNOutput_limit(void)
{
	//PWM限幅�?0.6A
    if(PWMInfo.PWMout[v_wheel1_speed] < 1300)  PWMInfo.PWMout[v_wheel1_speed] = 1300;//反转还会�?放大
    if(PWMInfo.PWMout[v_wheel1_speed] > 1700)  PWMInfo.PWMout[v_wheel1_speed] = 1700;

    if(PWMInfo.PWMout[v_wheel2_speed] < 1300)  PWMInfo.PWMout[v_wheel2_speed] = 1300;
    if(PWMInfo.PWMout[v_wheel2_speed] > 1700)  PWMInfo.PWMout[v_wheel2_speed] = 1700;
	
    if(PWMInfo.PWMout[v_wheel3_speed] < 1300)  PWMInfo.PWMout[v_wheel3_speed] = 1300;
    if(PWMInfo.PWMout[v_wheel3_speed] > 1700)  PWMInfo.PWMout[v_wheel3_speed] = 1700;
	
    if(PWMInfo.PWMout[v_wheel4_speed] < 1300)  PWMInfo.PWMout[v_wheel4_speed] = 1300;
    if(PWMInfo.PWMout[v_wheel4_speed] > 1700)  PWMInfo.PWMout[v_wheel4_speed] = 1700;
	
    // if(PWMInfo.PWMout[h_wheel1_speed] < 1300)  PWMInfo.PWMout[h_wheel1_speed] = 1300;
    // if(PWMInfo.PWMout[h_wheel1_speed] > 1700)  PWMInfo.PWMout[h_wheel1_speed] = 1700;
	
    // if(PWMInfo.PWMout[h_wheel2_speed] < 1300)  PWMInfo.PWMout[h_wheel2_speed] = 1300;
    // if(PWMInfo.PWMout[h_wheel2_speed] > 1700)  PWMInfo.PWMout[h_wheel2_speed] = 1700;
	
    // if(PWMInfo.PWMout[h_wheel3_speed] < 1300)  PWMInfo.PWMout[h_wheel3_speed] = 1300;
    // if(PWMInfo.PWMout[h_wheel3_speed] > 1700)  PWMInfo.PWMout[h_wheel3_speed] = 1700;
	
    // if(PWMInfo.PWMout[h_wheel4_speed] < 1300)  PWMInfo.PWMout[h_wheel4_speed] = 1300;
    // if(PWMInfo.PWMout[h_wheel4_speed] > 1700)  PWMInfo.PWMout[h_wheel4_speed] = 1700;
}

