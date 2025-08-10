#include "task_conf.h"

#include "config.h"

extern PWMInfo_T PWMInfo;
/**
 * @brief 电机初始化
 * @param null
 * @retval null
*/
void Task_Thruster_Init(void)
{
	Drv_Delay_Ms(6000);		/* 初始停转信号后等待稳定 */
	
	/* 停转，1500ms(7.5%占空比) */
	Drv_PWM_HighLvTimeSet(&PWM[0], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[1], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[2], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[3], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[4], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[5], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[6], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[7], STOP_PWM_VALUE);
	Drv_Delay_Ms(2000);
}

/**
 * @brief 电机转速设置子函数
 * @param index 选择几号推进器
 * @param _Htime 高电平时间	_Htime可设置为500-2500,1500停止
							500-1500 逆时针旋转 1500-2500顺时针旋转
 * @retval null
*/
void Task_Thruster_SpeedSet(int index,uint16_t _Htime)
{
	Drv_PWM_HighLvTimeSet(&PWM[index], _Htime);
}

/**
 * @brief 所有电机相同转速设置函数
 * @param _Htime 高电平时间	_Htime可设置为500-2500,1500停止
							500-1500 逆时针旋转 1500-2500顺时针旋转
 * @retval null
*/
void Task_Thruster_AllSpeedSet(uint16_t _HTime)
{
	Drv_PWM_HighLvTimeSet(&PWM[0], _HTime);
	Drv_PWM_HighLvTimeSet(&PWM[1], _HTime);
	Drv_PWM_HighLvTimeSet(&PWM[2], _HTime);
	Drv_PWM_HighLvTimeSet(&PWM[3], _HTime);
}	

/**
 * @brief 电机开始工作
 * @param index 选择几号推进器
 * @param _Htime 高电平时间
 * @retval null
*/
void Task_Thruster_Start(int index,uint16_t _HTime)
{
	/* 电机运行速度设置 */
	Task_Thruster_SpeedSet(index,_HTime);
}

/**
 * @brief 所有推进器开始工作
 * @param *adress 存放八个推进器的分量数组的首地址
 * @retval null
*/
void Task_Thruster_AllStart(uint16_t *adress)
{
	Task_Thruster_Start(0,*adress);
	Task_Thruster_Start(1,*(adress+1));
	Task_Thruster_Start(2,*(adress+2));
	Task_Thruster_Start(3,*(adress+3));
	Task_Thruster_Start(4,*(adress+4));
	Task_Thruster_Start(5,*(adress+5));
	Task_Thruster_Start(6,*(adress+6));
	Task_Thruster_Start(7,*(adress+7));
}

/**
 * @brief 水平推进器开始工作
 * @param *adress 存放八个推进器的分量数组的首地址
 * @retval null
*/
void Task_Thruster_pin_AllStart(uint16_t *adress)
{
	Task_Thruster_Start(0,*adress);
	Task_Thruster_Start(1,*(adress+1));
	Task_Thruster_Start(2,*(adress+2));
	Task_Thruster_Start(3,*(adress+3));
}



/**
 * @brief 电机停止工作
 * @param index 选择几号推进器
 * @retval null
*/
void Task_Thruster_Stop(int index)
{
	/* 电机运行速度设置 */
	Drv_PWM_HighLvTimeSet(&PWM[index], STOP_PWM_VALUE);
	PWMInfo.PWMout[index] = STOP_PWM_VALUE;
}

/**
 * @brief 所有电机停止工作
 * @param null
 * @retval null
*/
void Task_Thruster_AllStop(void)
{
	Task_Thruster_Stop(0);
	Task_Thruster_Stop(1);
	Task_Thruster_Stop(2);
	Task_Thruster_Stop(3);
	Task_Thruster_Stop(4);
	Task_Thruster_Stop(5);
	Task_Thruster_Stop(6);
	Task_Thruster_Stop(7);
}

/**
 * @brief 角度到高电平时间，给舵机函数使用
 * @param angle 角度参数
 * @retval high_time 高电平时间
*/
uint16_t Servo_Angle_To_HightTime(uint16_t angle)
{
	uint16_t _HTime;
	//求出舵机角度对应的带宽，每9度多50带宽，360就是多2000带宽
	_HTime = angle*50/9 + 500;
	//限幅
	if(_HTime>2500) _HTime = 2500;
	if(_HTime<500) _HTime = 500;
	//返回高电平时间
	return _HTime;
}

/**
 * @brief 所有舵机和灯光开始工作
 * @param *adress 存放八个推进器和三个舵机和两个灯的分量数组的首地址
 * @retval null
*/
void Task_Servo_AllStart(uint16_t *adress)
{
	Task_Thruster_Start(8,*(adress+8));
	Task_Thruster_Start(9,*(adress+9));
	Task_Thruster_Start(10,*(adress+10));
	Task_Thruster_Start(11,*(adress+11));
	//Task_Thruster_Start(12,*(adress+12));
}

/**
 * @brief 所有舵机回到初始角度
 *   @param null
 * @retval null
*/
void Task_Servo_AllStop(void)
{
	//舵机应有的平衡角度
	Drv_PWM_HighLvTimeSet(&PWM[8], Servo_Angle_To_HightTime(CLAW_SHOUDER_STOP_ANGLE));
	Drv_PWM_HighLvTimeSet(&PWM[9], Servo_Angle_To_HightTime(CLAW_ELBOW_STOP_ANGLE));
	Drv_PWM_HighLvTimeSet(&PWM[10], Servo_Angle_To_HightTime(CLAW_CATCH_STOP_ANGLE));
}

