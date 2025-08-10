/****************************************************************************

* MATE团队

* 文件名: algo_kalman.c

* 内容简述：kalman滤波相关函数

* 文件历史：

* 版本号		日期	    作者		    说明
* V1.1.0 	2025-03-16	 黄迦南		 创建该文件

****************************************************************************/
#include "drv_hal_conf.h"
#include "config.h"
#include "stdint.h"  // 添加标准整数头文件，确保 int16_t 已定义
#include "algo_kalman.h"
// 定义滤波器实例
kalman_controller R_encoderkal;
kalman_controller L_encoderkal;
lpf_controller lpf_controller_l, lpf_controller_r;


// 一阶惯性滤波器初始化
void lpf_init(lpf_controller* lpf) {
    lpf->last_out = 0;
}

//一阶低通滤波器更新
int16_t lpf_update(lpf_controller* lpf, int16_t input) {
    // 更新滤波器输出，进行加权平均
    return lpf->last_out = (int16_t)(lpf->alpha * input + (1 - lpf->alpha) * lpf->last_out);
}

// 卡尔曼滤波器更新
float kalman_update(kalman_controller *klm, float input) {
    // 预测协方差方程
    klm->Now_P = klm->LastP + klm->Q;
    // 计算卡尔曼增益
    klm->Kg = klm->Now_P / (klm->Now_P + klm->R);
    // 更新最优估计
    klm->out = klm->out + klm->Kg * (input - klm->out);
    // 更新协方差
    klm->LastP = (1 - klm->Kg) * klm->Now_P;
    return klm->out;
}

// 卡尔曼滤波器初始化
void kalman_init(kalman_controller *klm, float klm_Q, float klm_R) {
    klm->LastP = 1.03;  // 初始协方差
    klm->Now_P = 0;     // 当前协方差
    klm->out = 0;       // 初始输出
    klm->Kg = 0;        // 初始卡尔曼增益
    klm->Q = klm_Q;     // 过程噪声协方差
    klm->R = klm_R;     // 测量噪声协方差
}
