/****************************************************************************

* MATE团队

* 文件名: algo_median_F.c

* 内容简述：中值滤波相关函数

* 文件历史：

* 版本号		日期	    作者		    说明
* V1.1.0 	2025-03-16	 黄迦南		 创建该文件

****************************************************************************/
#include "drv_hal_conf.h"
#include "config.h"
#include "algo_median_F.h"
float getMedian(float *buffer, int size)
{
    float temp[size]; 
    memcpy(temp, buffer, size * sizeof(float)); 
    qsort(temp, size, sizeof(float), compareFloat); 
    return temp[size / 2]; 
}


int compareFloat(const void *a, const void *b)
{
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

float Median_Flitering_Output(float Curr_Depth)
{
    float depthBuffer[FILTER_WINDOW_SIZE] = {0}; // ??????
    int bufferIndex = 0;  // ??????
    int dataCount = 0;  // ????????
     // ?????
     depthBuffer[bufferIndex] = Curr_Depth;
     bufferIndex = (bufferIndex + 1) % FILTER_WINDOW_SIZE;  // ??????
     if (dataCount < FILTER_WINDOW_SIZE)
     dataCount++;  // ???????

     // ????
     float depthFiltered = getMedian(depthBuffer, dataCount);
     return depthFiltered;
}
