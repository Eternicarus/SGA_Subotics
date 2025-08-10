#ifndef __ALGO_PID_UPDATE1_H_
#define __ALGO_PID_UPDATE1_H_
typedef struct {
    float kp, ki, kd;   // PID增益系数
    float imax;         // 积分项最大值，防止积分饱和
    float integrator;   // 积分项
    float last_error;   // 上次误差值
    float last_derivative; // 上次微分项值
    uint32_t last_t;    // 上次计算时间
    float fCut;         // 微分低通滤波参数
    float scaler;       //增大倍数 
} PID_T;
#define M_PI 3.141
void reset_I(PID_T *pid);
float get_pid(PID_T *pid, float error);
void reset_PID(PID_T *pid);
void PID_Init(PID_T *pid);
void init_millis(void);
uint32_t millis(void);
void PID_Update(PID_T*_tPid,float _faPID[3]);


#endif
