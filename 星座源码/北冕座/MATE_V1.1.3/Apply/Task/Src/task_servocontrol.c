#include "task_conf.h"
#include "usercode.h"
#include "config.h"
/*舵机初始化函数*/
void Servo_Init(PWMInfo_T *t_PWM)
{
	t_PWM -> PWMout[claw_servo_down] = 2000;
	t_PWM -> PWMout[claw_servo_elbow] = 1500;
	t_PWM -> PWMout[claw_servo_up] = 925;
	t_PWM -> PWMout[claw_servo_gimbal] = 1500;
    t_PWM -> PWMout[claw_servo_shoulder] = 1290;
	t_PWM -> PWMout[claw_servo_syringe] = 550;
	Mode_control |= 0x80;      //Mode_control第八位为标志位初始位	
}


/*舵机改值配置函数*/
void Servo_Write(PWMInfo_T *t_PWM)
{
    switch (Mode_control)
    {
        // 下机械爪控制
        case 3:
//             t_PWM->PWMout[claw_servo_down] = 2100;
            t_PWM->PWMout[claw_servo_down] -= CLAW_STEP;
			 printf("down %d\r\n",t_PWM->PWMout[claw_servo_down]);
            break;
        case 1:
//             t_PWM->PWMout[claw_servo_down] = 1550;
            t_PWM->PWMout[claw_servo_down] += CLAW_STEP;
			 printf("down %d\r\n",t_PWM->PWMout[claw_servo_down]);
            break;

        // 机械臂小臂角度控制
        case 2:
            t_PWM->PWMout[claw_servo_elbow] += ELBOW_STEP;
		  printf("shoulder %d\r\n",t_PWM->PWMout[claw_servo_elbow]);
            break;
        case 4:
            t_PWM->PWMout[claw_servo_elbow] -= ELBOW_STEP;
			printf("shoulder %d\r\n",t_PWM->PWMout[claw_servo_elbow]);
            break;

        // 摄像头云台角度控制
        case 5:
            t_PWM->PWMout[claw_servo_gimbal] += CLAW_STEP;
            break;
        case 6:
            t_PWM->PWMout[claw_servo_gimbal] -= CLAW_STEP;
            break;

        // 顶部爪箱控制
        case 7:
            t_PWM->PWMout[claw_servo_up] += UP_STEP;
//			t_PWM->PWMout[claw_servo_up] -= CLAW_STEP;
			 printf("up %d\r\n",t_PWM->PWMout[claw_servo_up]);
            break;
        case 8:
            t_PWM->PWMout[claw_servo_up] -= UP_STEP;
//			t_PWM->PWMout[claw_servo_up] += CLAW_STEP;
			 printf("up %d\r\n",t_PWM->PWMout[claw_servo_up]);
            break;
		
		case 9:
            t_PWM->PWMout[claw_servo_shoulder] -= SHOULDER_STEP;
//			t_PWM->PWMout[claw_servo_up] -= CLAW_STEP;
			 printf("elbow %d\r\n",t_PWM->PWMout[claw_servo_shoulder]);
            break;
        case 10:
            t_PWM->PWMout[claw_servo_shoulder] += SHOULDER_STEP;
			printf("elbow %d\r\n",t_PWM->PWMout[claw_servo_shoulder]);
			break;
		case 11:
            t_PWM->PWMout[claw_servo_syringe] += SYRINGE_STEP;
//			t_PWM->PWMout[claw_servo_up] -= CLAW_STEP;
			 printf("syringe %d\r\n",t_PWM->PWMout[claw_servo_syringe]);
            break;
        case 12:
            t_PWM->PWMout[claw_servo_syringe] -= SYRINGE_STEP;
			printf("syringe %d\r\n",t_PWM->PWMout[claw_servo_syringe]);
			break;

        default:
            break;
    }
}


/*舵机限幅操作函数*/
void Servo_Limit(PWMInfo_T *t_PWM)
{   
	//下机械爪
	 if(t_PWM->PWMout[claw_servo_down] < 1600)   
	 t_PWM->PWMout[claw_servo_down] = 1600;
    
	 if(t_PWM->PWMout[claw_servo_down] > 2500) 
	 t_PWM->PWMout[claw_servo_down] = 2500;
    //小臂
	if(t_PWM->PWMout[claw_servo_elbow] < 600) 
	t_PWM->PWMout[claw_servo_elbow] = 600; 

	if(t_PWM->PWMout[claw_servo_elbow] > 2400) 
	t_PWM->PWMout[claw_servo_elbow] = 2400;
    //云台
	if(t_PWM->PWMout[claw_servo_gimbal] > 2200) 
	t_PWM->PWMout[claw_servo_gimbal] = 2200;

	if(t_PWM->PWMout[claw_servo_gimbal] < 800) 
	t_PWM->PWMout[claw_servo_gimbal] = 800;
	
	if(t_PWM->PWMout[claw_servo_up] > 1830) 
	t_PWM->PWMout[claw_servo_up] = 1830;

	if(t_PWM->PWMout[claw_servo_up] < 1000) 
	t_PWM->PWMout[claw_servo_up] = 1000;
	
	if(t_PWM->PWMout[claw_servo_shoulder] > 1575) 
	t_PWM->PWMout[claw_servo_shoulder] = 1575;

	if(t_PWM->PWMout[claw_servo_shoulder] < 765) 
	t_PWM->PWMout[claw_servo_shoulder] = 765;
	
	if(t_PWM->PWMout[claw_servo_syringe] > 1950) 
	t_PWM->PWMout[claw_servo_syringe] = 1950;

	if(t_PWM->PWMout[claw_servo_syringe] < 600) 
	t_PWM->PWMout[claw_servo_syringe] = 600;
	
}
