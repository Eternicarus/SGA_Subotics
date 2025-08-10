/****************************************************************************

* MATE团队

* 文件名: algo_sqrt.c

* 内容简述：开方控制相关函数

* 文件历史：

* 版本号		日期	    作者		    说明
* V1.1.0 	2025-03-16	   黄迦南		 创建该文件

****************************************************************************/
#include "drv_hal_conf.h"
#include "config.h"
#include "algo_sqrt.h"
float sign(float x) {
    return (x > 0) - (x < 0);
}
float safe_sqrt(float x) {
    return sqrtf(fmaxf(x, 0.0f));  // 防止 sqrt(负数)
}

float sqrt_controller(float error, tagPID_T *_tPID) {
    float dt = _tPID->dt;
    float second_ord_lim = _tPID->second_ord_lim;
    float p = _tPID->fKp;

    // 防止 dt 和 second_ord_lim 取 0
    if (dt <= 0.0f) dt = 0.001f;
    if (second_ord_lim < 1e-6f) second_ord_lim = 1e-6f;

    float correction_rate;
    
    if (second_ord_lim <= 0.0f) {
        correction_rate = error * p;
    } 
    else if (p == 0.0f) {
        correction_rate = sign(error) * safe_sqrt(2.0f * second_ord_lim * fabsf(error));
    } 
    else {
        float linear_dist = second_ord_lim / (p * p);
        if (fabsf(error) > linear_dist) {
            correction_rate = sign(error) * safe_sqrt(2.0f * second_ord_lim * (fabsf(error) - (linear_dist / 2.0f)));
        } else {
            correction_rate = error * p;
        }
    }

    // // 防止过冲 & NaN 计算
    // if (isnan(correction_rate) || isinf(correction_rate)) {
    //     correction_rate = 0.0f;
    // }
    // correction_rate = fmaxf(fminf(correction_rate, fabsf(error) / dt), -fabsf(error) / dt);
    if(correction_rate == NAN)
    {
        return 1;
    }
    
    return correction_rate;
}


