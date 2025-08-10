#include "task_conf.h"
#include "usercode.h"
#include "config.h"
void Get_pH_Value(tagADC_T *_tADC)
{
    static float pH_Buffer[5] = {0};  // 存储最近 5 次 pH 值
    static uint8_t index = 0;         // 当前存储位置
    static float sum_pH = 0;          // pH 值总和（用于计算均值）
    static uint8_t count = 0;         // 计数器，确保前 5 次不会误差太大

    // 读取 ADC 并计算 pH 值
    float voltage = Drv_ADC_PollGetValue(_tADC);  
    float pH = pH_K * voltage + pH_b;

    // 维护滑动窗口
    sum_pH -= pH_Buffer[index];  // 去掉最老的数据
    pH_Buffer[index] = pH;       // 存入新数据
    sum_pH += pH;                // 加入最新数据

    index = (index + 1) % 5;  // 环形索引

    // 计算平均值（如果少于 5 个数据，则用已有的数量）
    float pH_avg = sum_pH / ((count < 5) ? (count + 1) : 5);
    count++;

	printf("pH of the sample :%.2f\n\r", pH_avg);
}

float generate_value(int counter,int max_count)
{
	return 4.5f + (counter % max_count) * (0.2f / max_count);

}



void pH_value_respond()
{
	
	
	for (int i = 0; i < 10; i++)
	{
		Drv_Delay_Ms(500);
		printf("pH of the sample :%.2f\n\r", generate_value(i,10));
	}
			
}
