#ifndef __TASK_ANALYSISDATA_H_
#define __TASK_ANALYSISDATA_H_

typedef struct 
{
    char ModeChange[15];    //存放模式切换命令
		char Change[15]; 
    float fNum[3];          //存放浮点数数据
    bool keyPressed;
}HandleModeInfo;

typedef struct 
{
    char ModeChange[15];    //存放模式切换命令
    //uint8_t Status;       //直线状态为0，转弯状态为1
    float BlackAngle;       //黑线偏移角度[-90,90],顺时针正、逆时针负
    float CenterShift;      //中心偏移距离[-5000,5000],上正、下负
}AutoModeInfo;

typedef struct
{
    float setDepth;         //设定深度
}DepthControlInfo;


void Task_AnalysisData(uint8_t *DataBuf);

#endif

