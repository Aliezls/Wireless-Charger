#include "Function.h"

/**
 * Copy
 */

/*******************************************************************************
 * @fn Init_lowPass_alpha
 * @brief 初始化低通滤波器滤波系数
 * @param filter 滤波器
 * @param ts 采用周期 单位s
 * @return fc 截至频率 单位hz
 ******************************************************************************/
low_pass_filter_t low_pass_filter1 = {0}; // 定义滤波器
low_pass_filter_t low_pass_filter2 = {0}; // 定义滤波器
low_pass_filter_t low_pass_filter3 = {0}; // 定义滤波器

void Init_lowPass_alpha(low_pass_filter_t *const filter, const float ts, const float fc)
{
    float b = 2 * M_PI * fc * ts;
    filter->ts = ts;
    filter->fc = fc;
    filter->lastYn = 0;
    filter->alpha = b / (b + 1);
}
/*******************************************************************************
 * @fn Low_pass_filter
 * @brief 低通滤波函数
 * @param data 采样数据
 * @return 滤波结果
 ******************************************************************************/
float Low_pass_filter(low_pass_filter_t *const filter, const float data)
{
    float tem = filter->lastYn + (filter->alpha * (data - filter->lastYn));
    filter->lastYn = tem;
    return tem;
}

/*End Copy*/

/**
 * 用户自定义常量
 */
const float Light_R_Limit_voltage = 2.8; // 光敏电阻分压判断阈值，用于区分光敏电阻是否被完全遮挡，这个数与物理环境相关,在暗的地方应该把这个数调大，避免误触发
const float Current_Limit_Value = 0.25;  // 电流采样判断MOS时的电流阈值，用于区分输出电流是否有效，也可以略微调大，避免后级金属杂物误触发
const int Aver_Sample_count = 8;         // 均值滤波时的采样次数
const float Current_error_b = 0.1;       // 电流采样，线性修正的常数项->y=kx+b
const int User_OK = 1;                   // 表示状态的三常数
const int User_NO = 0;
const int User_Wait = -1;
/*End User*/

/**
 * 用户自定义变量
 */
ADC_Sample my_adc_value = {0}; // ADC采样结构体变量
MOS_State Mos = {0, 0, 0};     // Mos状态结构体
/*End User*/

/**
 * @brief 均值滤波，不过写得一坨
 * @note 现在的逻辑是，只有读取到count次数据后计算平均值赋给平均值变量，超过count次之后平均值变量不变，和置零
 * @note 所以程序运行时，平均值变量在该函数运行count次后才会更新,所以实际上对数据的更新做了降频处理，同时取平均
 * @note 不在while里面执行该函数，就不会阻塞在这个函数里面
 * @param[in] data 传入的ADC数据结构体
 * @param[in] count 均值采样次数
 * @param[in] n 0表示光敏电阻,1表示电压,2表示电流，3表示内部参考电压
 */
void Average_Flitter(ADC_Sample *myadc, int count, int n)
{
    myadc->sum[n] += myadc->ADC_Trs_value[n];
    myadc->count[n]++;
    if (myadc->count[n] >= count)
    {
        myadc->count[n] = 0;
        myadc->aver[n] = myadc->sum[n] / count;
        myadc->sum[n] = 0;
        myadc->Fliiter_state[n] = User_OK; // 表示滤波已经完成
    }
}

/**
 * @brief 开启ADC采样
 * @note 输入ADC指针，校准ADC通道并开启DMA数据传输
 * @param[in] adcp ADC操作通道指针
 */
void ADC_Sample_Start(ADC_HandleTypeDef *adcp)
{
    HAL_ADCEx_Calibration_Start(adcp); // 校准ADC采样通道

    HAL_ADC_Start_DMA(adcp, (uint32_t *)my_adc_value.ADC_values, sizeof(my_adc_value.ADC_values) / sizeof(uint16_t)); // 开启DMA
}

/**
 * @brief 转换ADC原始数据为标准单位
 * @note 输入自定的ADC_Sample结构体指针，进行转换
 * @note 转换和滤波的次序对最终结果影响比较大，
 * @param[in] myadc ADC数据结构体的指针
 */
void ADC_Data_Trans(ADC_Sample *myadc)
{
    myadc->Ref_Voltage = 4095.0 * 1.2 / myadc->ADC_values[3]; // 计算用于转换的芯片内部参考电压

    // myadc->ADC_Trs_value[0] = Low_pass_filter(&low_pass_filter1, myadc->ADC_Trs_value[0]); // 低通滤波
    myadc->ADC_Trs_value[0] = myadc->ADC_values[0] * myadc->Ref_Voltage / 4095.0; // 光敏电阻电压
    Average_Flitter(&my_adc_value, Aver_Sample_count, 0);                         // 对光敏电阻电压的均值滤波

    // myadc->ADC_Trs_value[1] = Low_pass_filter(&low_pass_filter2, myadc->ADC_Trs_value[1]); // 低通滤波
    myadc->ADC_Trs_value[1] = myadc->ADC_values[1] * myadc->Ref_Voltage / 4095.0;
    myadc->ADC_Trs_value[1] = myadc->ADC_Trs_value[1] / 15.0; // 根据运放的比例转化成相应电压值
    //这里的输入电压其实程序中并没有使用，不过可以做一个欠压关断保护

    // myadc->ADC_Trs_value[2] = Low_pass_filter(&low_pass_filter3, myadc->ADC_Trs_value[2]); // 低通滤波
    myadc->ADC_Trs_value[2] = myadc->ADC_values[2] * myadc->Ref_Voltage / 4095.0;
    myadc->ADC_Trs_value[2] = (myadc->ADC_Trs_value[2] - 1.65) * 10.0; // 根据霍尔电流传感器的采样范围(-10A~10A)，将电压转化成电流值
    myadc->ADC_Trs_value[2] += Current_error_b;                        // 尝试修正误差,这里没有修正K，因为实际结果好像还可以，暂时没写
    Average_Flitter(&my_adc_value, Aver_Sample_count, 2); // 电流的均值滤波

    if (myadc->ADC_Trs_value[2] < 0)
    {
        myadc->ADC_Trs_value[2] = 0; // 不可能存在负电流，故限制电流在0A以上
    }
    if (myadc->aver[2] < 0)
    {
        myadc->aver[2] = 0; // 不可能存在负平均电流，故限制电流在0A以上，但是实际好像上面限制后，就不存在负平均电流了
    }
}

/**
 * @brief 根据光敏电阻状态判断是否打开MOS管
 * @param[in] myadc 传入ADC数据指针，用于判断光敏电阻状态
 * @param[in] mos 传入MOS结构体状态指针，判断并更新光敏电阻状态
 */
int Judge_byLight(ADC_Sample *myadc, MOS_State *mos)
{
    if (myadc->Fliiter_state[0] == 1) // 光敏电阻分压值均值过滤完成
    {
        if (mos->LightR_state == User_NO && myadc->ADC_Trs_value[0] >= Light_R_Limit_voltage) // 如果刚才光敏电阻前面没有东西，且当前光敏电阻分压大于某个值(光强小于某个值)
        {
            mos->LightR_state = User_Wait;                                         // 更新光敏电阻状态，-1表示有物体经过，但是无法判断是否停在面前
            HAL_GPIO_WritePin(MOS_SWITCH_GPIO_Port, MOS_SWITCH_Pin, GPIO_PIN_SET); // 打开MOS，以供后续电流检测是否继续开启MOS
            HAL_GPIO_WritePin(Light_SW_GPIO_Port, Light_SW_Pin, GPIO_PIN_RESET);   // 打开白灯，表示进入识别模式，如果前面有东西，但不是线圈，这个灯就会300ms闪烁一下
            HAL_Delay(300);                                                        // 略微延时，让电流能涨上去,同时缓冲一下，方便下次再进行检测，避免对于忽然经过光敏电阻前的物体误反应
            return User_Wait;
        }
        else if (mos->LightR_state == User_NO && myadc->ADC_Trs_value[0] < Light_R_Limit_voltage)
        {
            mos->LightR_state = User_NO; // 更新光敏电阻状态，0表示目前没有东西在面前
        }
        else if (mos->LightR_state == User_Wait && myadc->ADC_Trs_value[0] >= Light_R_Limit_voltage) // 如果上一次电阻前面疑似有东西，那么这次就是再次检测
        {
            mos->LightR_state = User_OK; // 如果还是有东西，那么可以确定就是有东西在前面，可以更新光敏电阻状态为1
        }
        else if (mos->LightR_state == User_Wait && myadc->ADC_Trs_value[0] < Light_R_Limit_voltage)
        {
            HAL_GPIO_WritePin(Light_SW_GPIO_Port, Light_SW_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOS_SWITCH_GPIO_Port, MOS_SWITCH_Pin, GPIO_PIN_RESET);
            mos->LightR_state = User_NO; // 如果疑似前面原有的东西消失了，那么就暂且认为东西闪过去了，取消识别模式，关闭MOS管
        }
        return User_OK;
    }
    else
    {
        return User_NO; // 表示滤波还没完成，读不到这次滤波完的数据，继续等待滤波
    }
}

/**
 * @brief 试着通过采样的电流大小判断MOS是否应该继续打开
 * @note 在光敏电阻检测的后面执行
 */
void Judge_byCur(ADC_Sample *myadc, MOS_State *mos)
{
    if (mos->LightR_state == User_OK) // 如果判断确有东西在前面，检测此时的电流
    {

        if (myadc->ADC_Trs_value[2] >= Current_Limit_Value) // 如果输入电流值大于某个值
        {
            Mos.Current_state = 1; // 说明线圈后级是接收线圈，可以放电
        }
        if (myadc->ADC_Trs_value[2] < Current_Limit_Value) // 如果输入电流值小于某个值
        {
            Mos.Current_state = 0; // 说明线圈后级没有东西，或者其他金属异物
        }
    }
    else if (mos->LightR_state == User_NO)
    { // 如果判断没有东西在前面
        mos->Current_state = 0;
    }
}

/**
 * @brief 判断是否打开MOS的函数
 * @note 结合光敏电阻和采样电流大小来判断是否持续打开MOS
 * @note 这里没用电流的均值滤波器状态，但程序跑起来也没问题，后续可以试验一下
 */
void Open_Mos(MOS_State *mos)
{
    Judge_byLight(&my_adc_value, &Mos); // 光敏电阻检测
    if (mos->LightR_state == User_OK)
    {
        Judge_byCur(&my_adc_value, &Mos); // 电流大小检测
    }

    if (mos->LightR_state == User_OK && mos->Current_state == User_OK)
    {
        HAL_GPIO_WritePin(MOS_SWITCH_GPIO_Port, MOS_SWITCH_Pin, GPIO_PIN_SET); // 保持MOS管的打开
    }
    else
    {
        HAL_GPIO_WritePin(MOS_SWITCH_GPIO_Port, MOS_SWITCH_Pin, GPIO_PIN_RESET); // 关闭MOS管
    }
}