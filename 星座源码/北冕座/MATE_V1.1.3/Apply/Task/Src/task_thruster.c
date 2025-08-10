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
	Drv_PWM_HighLvTimeSet(&PWM[v_wheel1_speed], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[v_wheel2_speed], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[v_wheel3_speed], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[v_wheel4_speed], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[h_wheel1_speed], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[h_wheel2_speed], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[h_wheel3_speed], STOP_PWM_VALUE);
	Drv_PWM_HighLvTimeSet(&PWM[h_wheel4_speed], STOP_PWM_VALUE);
	// Drv_PWM_HighLvTimeSet(&PWM[8], STOP_PWM_VALUE);
	// Drv_PWM_HighLvTimeSet(&PWM[9], 1000);
	// Drv_PWM_HighLvTimeSet(&PWM[10], STOP_PWM_VALUE);
	// Drv_PWM_HighLvTimeSet(&PWM[11], 1250);
	//使舵机处于初始设定位置
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
	Task_Thruster_Start(v_wheel1_speed,*(adress+v_wheel1_speed));
	Task_Thruster_Start(v_wheel2_speed,*(adress+v_wheel2_speed));
	Task_Thruster_Start(v_wheel3_speed,*(adress+v_wheel3_speed));
	Task_Thruster_Start(v_wheel4_speed,*(adress+v_wheel4_speed));
	Task_Thruster_Start(h_wheel1_speed,*(adress+h_wheel1_speed));
	Task_Thruster_Start(h_wheel2_speed,*(adress+h_wheel2_speed));
	Task_Thruster_Start(h_wheel3_speed,*(adress+h_wheel3_speed));
	Task_Thruster_Start(h_wheel4_speed,*(adress+h_wheel4_speed));
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
 * @brief 垂直推进器开始工作
 * @param *adress 存放八个推进器的分量数组的首地址
 * @retval null
*/
void Task_Thruster_chui_AllStart(uint16_t *adress)
{
	Task_Thruster_Start(4,*(adress+4));
	Task_Thruster_Start(5,*(adress+5));
	Task_Thruster_Start(6,*(adress+6));
	Task_Thruster_Start(7,*(adress+7));
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
	Task_Thruster_Start(claw_servo_down,*(adress+claw_servo_down));
	Task_Thruster_Start(claw_servo_shoulder,*(adress+claw_servo_shoulder));
	Task_Thruster_Start(claw_servo_up,*(adress+claw_servo_up));
	Task_Thruster_Start(claw_servo_gimbal,*(adress+claw_servo_gimbal));
	Task_Thruster_Start(claw_servo_syringe,*(adress+claw_servo_syringe));
    Task_Thruster_Start(claw_servo_elbow,*(adress+claw_servo_elbow));
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

