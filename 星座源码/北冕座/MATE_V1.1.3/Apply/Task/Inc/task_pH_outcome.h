#ifndef __TASK_DEPTHCONTROL_H_
#define __TASK_DEPTHCONTROL_H_

#define pH_K -5.8402
#define pH_b 16.733

#include "task_conf.h"
#include "usercode.h"
#include "config.h"


void Get_pH_Value(tagADC_T *_tADC);
void pH_value_respond();


#endif
