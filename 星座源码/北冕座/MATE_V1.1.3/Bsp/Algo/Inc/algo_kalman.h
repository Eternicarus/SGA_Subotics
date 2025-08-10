#ifndef KALMAN_H_
#define KALMAN_H_
#include  "drv_hal_conf.h"
#include <stdint.h> // 添加标准整数库，确保数据类型明确

// 一阶低通滤波器LPF
typedef struct {
    float alpha;  // 滤波系数
    int16_t last_out;  // 上次滤波输出
} lpf_controller;

// 函数声明
void lpf_init(lpf_controller* lpf);  // 初始化低通滤波器
int16_t lpf_update(lpf_controller* lpf, int16_t input);  // 更新低通滤波器

// 卡尔曼滤波器KF
typedef struct {
    float LastP;  // 上次估算协方差
    float Now_P;  // 当前估算协方差
    float out;    // 卡尔曼滤波器输出
    float Kg;     // 卡尔曼增益
    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差
} kalman_controller;

// 函数声明
void kalman_init(kalman_controller *klm, float klm_Q, float klm_R);  // 初始化卡尔曼滤波器
float kalman_update(kalman_controller *klm, float input);  // 更新卡尔曼滤波器

// 外部变量声明
extern kalman_controller R_encoderkal;
extern kalman_controller L_encoderkal;

extern lpf_controller lpf_controller_l, lpf_controller_r;
extern lpf_controller lpf_load;

extern kalman_controller kalman_controller_q, kalman_controller_h, kalman_controller_d3, kalman_controller_d4;
extern kalman_controller kalman_err_q, kalman_err_h, kalman_err_d, kalman_sensor[7];

extern float q_Q, q_R;
extern float h_Q, h_R;
extern float d_Q, d_R;

#endif /* KALMAN_H_ */
