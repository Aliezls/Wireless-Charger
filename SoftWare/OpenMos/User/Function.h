#ifndef FUNCTION_H
#define FUNCTION_H

#include "adc.h"
#include "main.h"
/**
 * Copy
 */
/****************************  低通滤波器相关定义  ********************************/
#ifndef M_PI
#define M_PI (3.141592f)
#endif
typedef struct
{
    float ts;     // 采样周期(s)
    float fc;     // 截至频率(hz)
    float lastYn; // 上一次滤波值
    float alpha;  // 滤波系数
} low_pass_filter_t;

// 初始化滤波系数
void Init_lowPass_alpha(low_pass_filter_t *const filter, const float ts, const float fc);
// 低通滤波
float Low_pass_filter(low_pass_filter_t *const filter, const float data);
/*************************** END of 低通滤波器相关定义 ***************************************************/

/*End Copy*/

/**
 * 存储ADC所有数据的结构体
 */
typedef struct
{
    /**
     * 0:光敏电阻分压
     * 1:输入电压
     * 2:输入电流
     * 3:内部参考电压
     */
    uint16_t ADC_values[4]; // 用于存储ADC采样数据的数组

    /**
     * 0:转换后表示光敏电阻分压，与光强大致反比
     * 1:输入电压(V)
     * 2:输入电流(A)
     */
    float ADC_Trs_value[3]; // 用于存储转换后的数据的数组

    float Ref_Voltage; // 用于数据转换的内部参考电压 =4095*1.2/ADC_values[3];

    /**
     * 用于均值滤波数据
     * 0:光敏电阻分压
     * 1:输入电压
     * 2:输入电流
     * 3:内部参考电压
     */
    float count[4];       // 表示当前已经采样过的次数
    float sum[4];         // 采样值总和
    float aver[4];        // 采样平均值
    int Fliiter_state[4]; // 表示是否滤波完成
    /*End User */
} ADC_Sample;

/**
 * 存储MOS开关状态相关的数据的结构体
 */
typedef struct
{
    int LightR_state;   // 表示当前光敏电阻是否被遮挡
    int Current_state;  // 表示当前电流是否足够，证明后级是接收线圈
    // int Cur_judge_flag; // 便于先多次采样转换电流，废弃

} MOS_State;

void ADC_Sample_Start(ADC_HandleTypeDef *adcp);            // ADC的DMA采样模式开启
void ADC_Data_Trans(ADC_Sample *myadc);                    // ADC数据转换
int Judge_byLight(ADC_Sample *myadc, MOS_State *mos);      // 更新光敏电阻状态
void Judge_byCur(ADC_Sample *myadc, MOS_State *mos);       // 更新电流状态
void Open_Mos(MOS_State *mos);                             // 根据MOS参数判断是否打开MOS
void Average_Flitter(ADC_Sample *myadc, int count, int n); // 均值滤波

#endif