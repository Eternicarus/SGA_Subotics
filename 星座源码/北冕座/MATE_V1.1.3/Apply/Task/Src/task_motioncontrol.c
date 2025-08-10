#include "task_conf.h"
#include "usercode.h"
#include "config.h"

extern PWMInfo_T PWMInfo;
extern DepthControlInfo DCInfo;

void HandleMode_data_handle(float ExpDepth)
{
	//各方向速度分量
	short x1_Speed = 0;
	short x2_Speed = 0;
	short x3_Speed = 0;
	short x4_Speed = 0;
	short y1_Speed = 0;
	short y2_Speed = 0;
	short y3_Speed = 0;
	short y4_Speed = 0;
	
	short YAWspeed = 0;
	short z_Speed = 0;
	float error;
	short Basic_Wheel_Speed =0;
	// short z_Speed = 0;
	
	
	//根据储存按键数据和扳机速度系数处理出各方向速度分量的大小
	//按键
	//慢速（仅前进后退左右）
	// if((x_y_z_pitch & 0x80) && (SpeedMode == 1))
	// 	x_Speed += SLOW_SPEED;
	// if((x_y_z_pitch & 0x40) && (SpeedMode == 1))
	//     x_Speed -= SLOW_SPEED;
	// if((x_y_z_pitch & 0x20) && (SpeedMode == 1))
	// 	y_Speed += SLOW_SPEED;
	// if((x_y_z_pitch & 0x10) && (SpeedMode == 1))
	//     y_Speed -= SLOW_SPEED;
	// if((left_rocker==3) && (SpeedMode == 1))
	// 	YAWspeed = SLOW_SPEED;
	// if((left_rocker==4) && (SpeedMode == 1))
	// 	YAWspeed = -SLOW_SPEED;
	
	//原速
	if(x_y_z_pitch & 0x80)      //左平移
	{  
		  x1_Speed = 350;        //350
	    x2_Speed = 120;        //120
	    x3_Speed = 350;          //300
	    x4_Speed = 140;          //100
	}
	if(x_y_z_pitch & 0x40)      //右平移
	{ 
		x1_Speed = -150;           //右后150
	    x2_Speed = -350;           //右前350
	    x3_Speed = -130;            //左前
	    x4_Speed = -350;            //左后
	}
	if(x_y_z_pitch & 0x20)      //前进
	{
		y1_Speed = 270;
	    y2_Speed = 300;
	    y3_Speed = 270;
	    y4_Speed = 270;
	}
	if(x_y_z_pitch & 0x10)      //后退
	{   
		y1_Speed = -300;
	    y2_Speed = -300;
	    y3_Speed = -300;
	    y4_Speed = -300;
	}
	if(left_rocker == 3)    
		YAWspeed = YAWSPEED;
	if(left_rocker == 4)
		YAWspeed = -YAWSPEED;
	if(DepthFlag & 0x02)
	{
//		error = ExpDepth - DepthFliter;
		z_Speed = -Algo_PID_Calculate(&DepthPID,DepthFliter,ExpDepth);
//		z_Speed = -sqrt_controller(error,&DepthPID);
//		printf("F:%f E:%f\r\n",MS5837.fDepth,ExpDepth);
		// printf("PIDout: %d\r\n",z_Speed);
	}
	else if(floatflag & 0x02)
	{
		z_Speed = -150;   //底部作业保持
	}
	else if(sinkflag & 0x02)
	{
		z_Speed = 150;  //顶部作业保持
	}
	
	if(x_y_z_pitch & 0x08)
	{
		z_Speed = 300;
	}
    else if(x_y_z_pitch & 0x04)
	{
	    z_Speed = -300;
	}
//	printf("%d\r\n",z_Speed);
	//计算推进器推力
	PWMInfo.PWMout[v_wheel1_speed] =  y1_Speed -  x1_Speed - YAWspeed  + STOP_PWM_VALUE;
    PWMInfo.PWMout[v_wheel2_speed] =  y2_Speed +  x2_Speed - YAWspeed  + STOP_PWM_VALUE;
	PWMInfo.PWMout[v_wheel3_speed] =  y3_Speed -  x3_Speed + YAWspeed + STOP_PWM_VALUE;
    PWMInfo.PWMout[v_wheel4_speed] =  y4_Speed +  x4_Speed + YAWspeed + STOP_PWM_VALUE;
	// 死区配置
	if(z_Speed == 0.0)
        Basic_Wheel_Speed = z_Speed + STOP_PWM_VALUE;
    if(z_Speed > 0.0)
	    Basic_Wheel_Speed = z_Speed + STOP_PWM_VALUE;
    if(z_Speed < 0.0)
	    Basic_Wheel_Speed = z_Speed + STOP_PWM_VALUE;
	

	PWMInfo.PWMout[h_wheel1_speed] = Basic_Wheel_Speed ;  
    PWMInfo.PWMout[h_wheel2_speed] = Basic_Wheel_Speed ; 
    PWMInfo.PWMout[h_wheel3_speed] = Basic_Wheel_Speed ;
    PWMInfo.PWMout[h_wheel4_speed] = Basic_Wheel_Speed ; 
//	调试用
//	PWMInfo.PWMout[h_wheel1_speed] = 1800;  
//    PWMInfo.PWMout[h_wheel2_speed] = 1800; 
//    PWMInfo.PWMout[h_wheel3_speed] = 1800;
//    PWMInfo.PWMout[h_wheel4_speed] = 1800; 
//	PWMInfo.PWMout[v_wheel1_speed] = 1800;
//    PWMInfo.PWMout[v_wheel2_speed] = 1800;
//	PWMInfo.PWMout[v_wheel3_speed] = 1800;//左后反转，原因不明，暂定推进器原因
//    PWMInfo.PWMout[v_wheel4_speed] = 1800;
//	printf("%d %f\r\n",Basic_Wheel_Speed,PitchPID_out);
}


/*反转放大*/
void Thruster_nagative_data_handle(float times_L,float times_R)
{
	if(PWMInfo.PWMout[v_wheel1_speed] < 1500) PWMInfo.PWMout[v_wheel1_speed] = 1500-(1500-PWMInfo.PWMout[v_wheel1_speed])*times_R;
	if(PWMInfo.PWMout[v_wheel2_speed] < 1500) PWMInfo.PWMout[v_wheel1_speed] = 1500-(1500-PWMInfo.PWMout[v_wheel1_speed])*times_R;
	if(PWMInfo.PWMout[v_wheel3_speed] < 1500) PWMInfo.PWMout[v_wheel1_speed] = 1500-(1500-PWMInfo.PWMout[v_wheel1_speed])*times_L;	
	if(PWMInfo.PWMout[v_wheel4_speed] < 1500) PWMInfo.PWMout[v_wheel1_speed] = 1500-(1500-PWMInfo.PWMout[v_wheel1_speed])*times_L;
	if(PWMInfo.PWMout[h_wheel1_speed] < 1500) PWMInfo.PWMout[h_wheel1_speed] = 1500-(1500-PWMInfo.PWMout[h_wheel1_speed])*times_L;
	if(PWMInfo.PWMout[h_wheel2_speed] < 1500) PWMInfo.PWMout[h_wheel2_speed] = 1500-(1500-PWMInfo.PWMout[h_wheel2_speed])*times_L;
	if(PWMInfo.PWMout[h_wheel3_speed] < 1500) PWMInfo.PWMout[h_wheel3_speed] = 1500-(1500-PWMInfo.PWMout[h_wheel3_speed])*times_L;	
	if(PWMInfo.PWMout[h_wheel4_speed] < 1500) PWMInfo.PWMout[h_wheel4_speed] = 1500-(1500-PWMInfo.PWMout[h_wheel4_speed])*times_L;
		
}
/*电机限幅*/
void PWNOutput_limit(void)
{
	//PWM限幅1.8A 
	//电机最大转速，舵机写在另一处 
	//反转还会放大
    if(PWMInfo.PWMout[v_wheel1_speed] < 1150)  PWMInfo.PWMout[v_wheel1_speed] = 1150;
    if(PWMInfo.PWMout[v_wheel1_speed] > 1850)  PWMInfo.PWMout[v_wheel1_speed] = 1850;

    if(PWMInfo.PWMout[v_wheel2_speed] < 1150)  PWMInfo.PWMout[v_wheel2_speed] = 1150;
    if(PWMInfo.PWMout[v_wheel2_speed] > 1850)  PWMInfo.PWMout[v_wheel2_speed] = 1850;
	
    if(PWMInfo.PWMout[v_wheel3_speed] < 1150)  PWMInfo.PWMout[v_wheel3_speed] = 1150;
    if(PWMInfo.PWMout[v_wheel3_speed] > 1850)  PWMInfo.PWMout[v_wheel3_speed] = 1850;
	
    if(PWMInfo.PWMout[v_wheel4_speed] < 1150)  PWMInfo.PWMout[v_wheel4_speed] = 1150;
    if(PWMInfo.PWMout[v_wheel4_speed] > 1850)  PWMInfo.PWMout[v_wheel4_speed] = 1850;
	
    if(PWMInfo.PWMout[h_wheel1_speed] < 1200)  PWMInfo.PWMout[h_wheel1_speed] = 1200;
    if(PWMInfo.PWMout[h_wheel1_speed] > 1800)  PWMInfo.PWMout[h_wheel1_speed] = 1800;
	
    if(PWMInfo.PWMout[h_wheel2_speed] < 1200)  PWMInfo.PWMout[h_wheel2_speed] = 1200;
    if(PWMInfo.PWMout[h_wheel2_speed] > 1800)  PWMInfo.PWMout[h_wheel2_speed] = 1800;
	
    if(PWMInfo.PWMout[h_wheel3_speed] < 1200)  PWMInfo.PWMout[h_wheel3_speed] = 1200;
    if(PWMInfo.PWMout[h_wheel3_speed] > 1800)  PWMInfo.PWMout[h_wheel3_speed] = 1800;
	
    if(PWMInfo.PWMout[h_wheel4_speed] < 1200)  PWMInfo.PWMout[h_wheel4_speed] = 1200;
    if(PWMInfo.PWMout[h_wheel4_speed] > 1800)  PWMInfo.PWMout[h_wheel4_speed] = 1800;
}

