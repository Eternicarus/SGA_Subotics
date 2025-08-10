/****************************************************************************

* MATE团队

* 文件名: algo_pid update.c

* 内容简述：位置式PID相关函数

* 文件历史：

* 版本号		日期	    作者		    说明
* V1.1.0 	2025-03-16	 黄迦南		 创建该文件

****************************************************************************/
#include "drv_hal_conf.h"
#include "algo_pid update.h"

// 初始化 SysTick 计时
void init_millis() {
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000); // 配置 1ms 触发一次
}

uint32_t millis() {
    return HAL_GetTick();  // 直接用 HAL 的毫秒计时
}

// 复位积分项
void reset_I(PID_T *pid) {
    pid->integrator = 0;
    pid->last_derivative = 0.0f; // 设为 NAN，表示无效数据
}

// 计算 PID 输出    ##加入scaler缩放因子，在合理的情况下调整响应力度(整体缩放scaler倍)##可以先给1.0
float get_pid(PID_T *pid, float error) {
    uint32_t tnow = millis();
    uint32_t dt = tnow - pid->last_t;
    float output = 0;
    float delta_time;

    // 检查时间间隔，防止误差积累过大
    if (pid->last_t == 0 || dt > 1000) {
        dt = 0;
        reset_I(pid); // 如果长时间未使用，清空积分项
    }
    pid->last_t = tnow;
    delta_time = (float)dt * 0.001f; // 转换为秒

    // 计算比例项
    float P = error * pid->kp;
    output += P;

    // 计算微分项（如果时间间隔大于 0）
    if ((fabsf(pid->kd) > 0) && (dt > 0)) {
        float derivative;

        if (isnan(pid->last_derivative)) {
            derivative = 0; // 避免初始时 D 项突变
            pid->last_derivative = 0;
        } else {
            derivative = (error - pid->last_error) / delta_time;
        }

        // 低通滤波器，去除高频噪声  一阶低通滤波计算公式
        float RC = 1 / (2 * M_PI * pid->fCut);
        derivative = pid->last_derivative +
                     ((delta_time / (RC + delta_time)) *
                      (derivative - pid->last_derivative));

        pid->last_error = error;
        pid->last_derivative = derivative;

        float D = pid->kd * derivative;
        output += D;
    }

    // 缩放比例项和微分项
    output *= pid->scaler;

    // 计算积分项（如果时间间隔大于 0）
    if ((fabsf(pid->ki) > 0) && (dt > 0)) {
        pid->integrator += (error * pid->ki) *  pid->scaler * delta_time;
        // 限制积分项，防止积分饱和   限幅
        if (pid->integrator < -pid->imax) {
            pid->integrator = -pid->imax;
        } else if (pid->integrator > pid->imax) {
            pid->integrator = pid->imax;
        }
        output += pid->integrator;
    } 
    return output;
}

// 复位 PID 控制器
void reset_PID(PID_T *pid) {
    memset(pid, 0, sizeof(PID_T));
    reset_I(pid);
}


// 初始化 PID 参数
void PID_Init(PID_T *pid) {
    reset_I(pid);
}
void PID_Update(PID_T*_tPid,float _faPID[3])
{
    /* 更新PID系数 */
    _tPid->kp = _faPID[0];
    _tPid->ki = _faPID[1];
    _tPid->kd = _faPID[2];
}

