#include "task_conf.h"
#include "usercode.h"
#include "config.h"

extern PWMInfo_T PWMInfo;

void Task_AutoMode_Process(AutoModeInfo AMInfo)
{
    float Angle = 0.0f;             //存储运动角度 [-180,180]
    float CurrYaw = 0.0f;           //当前艏向角来自JY901S
    float ExpYaw = 0.0f;            //期望艏向角
    float AngleLoopPIDOut = 0.0;    //角度环PID计算后的结果

    float Offset = 0.0f;            //直线偏移量 [-100,100]
    float PositionLoopPIDOut = 0.0; //位置环PID计算后的结果

    float Wheel = 0; //方向增量
    float Pedal = 0; //推进增量

    Angle = AMInfo.BlackAngle;
    //设定当前角度和期望角度
    CurrYaw = JY901S.stcAngle.ConYaw;
    ExpYaw = CurrYaw - Angle;

    //角度环PID运算
    AngleLoopPIDOut = Algo_PID_Calculate(&AngleLoopPID,CurrYaw,ExpYaw);
    if(AngleLoopPIDOut > 90) AngleLoopPIDOut = 90;
    if(AngleLoopPIDOut < -90) AngleLoopPIDOut = -90;
    printf("AngleLoopPIDOut %f\r\n",AngleLoopPIDOut);

    //存储位置偏移
    Offset = AMInfo.CenterShift / 50;

    //位置环PID运算
    PositionLoopPIDOut = Algo_PID_Calculate(&PositionLoopPID,Offset,0);
    if(PositionLoopPIDOut > 100) PositionLoopPIDOut = 100;
    if(PositionLoopPIDOut < -100) PositionLoopPIDOut = -100;
    printf("PositionLoopPIDOut %f\r\n",PositionLoopPIDOut);

    if(Offset < 0)    //需要急转弯回到原位
    {
        if(Angle >= 0)
        {
            //Angle越大,Pedal越小,wheel越大
            //Offset的绝对值越大，Pedal越小,wheel越大
            Wheel = A * AngleLoopPIDOut * (1.0f + B * PositionLoopPIDOut/100.0f);
            Pedal = 1000.0f/(D * PositionLoopPIDOut * (1.0f + C * fabs(AngleLoopPIDOut)/90.f));
        }
        else
        {
            //Angle的绝对值越大,Pedal越小,wheel越大
            //Offset的绝对值越大，Pedal越小,wheel越大
            Wheel = A * AngleLoopPIDOut * (1.0f + B * PositionLoopPIDOut/100.0f);
            Pedal = 1000.0f/(D * PositionLoopPIDOut* (1.0f + C * AngleLoopPIDOut/90.f));      
        }
    }
    else
    {
        if(Angle >= 0)
        {
            //Angle越大,Pedal越小,wheel越大
            //Offset越大，Pedal越大,wheel越小
            Wheel = A * AngleLoopPIDOut * (1.0f - fabs(B * PositionLoopPIDOut)/100.0f);
            Pedal = - D * PositionLoopPIDOut /((1.0f + C * fabs(AngleLoopPIDOut)/90.f));
        }
        else
        {
            //Angle的绝对值越大,Pedal越小,wheel越大
            //Offset越大，Pedal越大,wheel越小
            Wheel = A * AngleLoopPIDOut * (1.0f - fabs(B * PositionLoopPIDOut)/100.0f);
            Pedal = - D * PositionLoopPIDOut /((1.0f + C * AngleLoopPIDOut/90.f));       
        }  
    }

    printf("Wheel:%f Pedal:%f\r\n",Wheel,Pedal);

    //wheel < 0,右转；wheel > 0,左转
    PWMInfo.PWMout[LeftHThruster] = Pedal - Wheel + STOP_PWM_VALUE + DEADBAND;
    PWMInfo.PWMout[RightHThruster] = Pedal + Wheel + STOP_PWM_VALUE + DEADBAND;

    printf("Left: %d Right:%d\r\n",PWMInfo.PWMout[LeftHThruster],PWMInfo.PWMout[RightHThruster]);
    //PWM限幅，0.6A
    if(PWMInfo.PWMout[LeftHThruster] < 1400)  PWMInfo.PWMout[LeftHThruster] = 1400;
    if(PWMInfo.PWMout[LeftHThruster] > 1600)  PWMInfo.PWMout[LeftHThruster] = 1600;

    if(PWMInfo.PWMout[RightHThruster] < 1400)  PWMInfo.PWMout[RightHThruster] = 1400;
    if(PWMInfo.PWMout[RightHThruster] > 1600)  PWMInfo.PWMout[RightHThruster] = 1600;

    //根据PID结果改变左右翼水平推进器PWM，假设1为左水平推，2为右水平推，超过1500为前进方向
    Task_Thruster_SpeedSet(LeftHThruster,PWMInfo.PWMout[LeftHThruster]);
    Task_Thruster_SpeedSet(RightHThruster,PWMInfo.PWMout[RightHThruster]);

    //printf("Auto left%d right%d\r\n",PWMInfo.PWMout[LeftHThruster],PWMInfo.PWMout[RightHThruster]);
}
