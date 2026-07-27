/**
  ******************************************************************************
  * @file           : app_current_sense.c
  * @brief          : 三相电流采样模块
  * @note           : ADC1 注入组 + JEOC 中断, 由 HRTIM ADC_TRG2 触发.
  *                   提供电流读取、数据就绪标志、零点校准功能.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "app_current_sense.h"

/* Private variables ---------------------------------------------------------*/

/** 电流缓冲: 3 相 Ia, Ib, Ic (由注入组 JEOC 中断更新) */
static uint16_t adc_curr_buf[3] = {0};

/** 各通道零点 ADC 偏移量 (校准后可运行时修改) */
static uint16_t adc_offset[3] = {
    CURR_SENSE_OFFSET_A,
    CURR_SENSE_OFFSET_B,
    CURR_SENSE_OFFSET_C,
};

/** ADC_SCALE_MA_PER_LSB = (Vref*1000) / (Resolution * Gain * Rshunt)  (mA/LSB) */
static const float scale_ma_per_lsb =
    (CURR_SENSE_ADC_VREF * 1000.0f) /
    (CURR_SENSE_ADC_RESOLUTION * CURR_SENSE_OPAMP_GAIN * CURR_SENSE_SHUNT_RESISTANCE);

/** 新数据就绪标志 (ADC 完成回调置 1, 读取后清 0) */
static volatile uint8_t new_data_flag = 0;

/** EMA 低通滤波系数 (0~1, 默认 0.15)
 *  alpha=1 直通无滤波, alpha 越小滤波越强.
 *  20kHz 采样率: alpha=0.15 -> fc≈500Hz; alpha=0.30 -> fc≈1.1kHz */
static float filter_alpha = 0.15f;

/** EMA 滤波后的电流值 (mA), 三相独立滤波 */
static float curr_filtered[3] = {0.0f, 0.0f, 0.0f};

/* Private function prototypes -----------------------------------------------*/

/**
 * \brief           将 ADC 原始值转换为电流 (mA)
 * \param[in]       adc_val  ADC 12-bit 原始值
 * \param[in]       channel  通道索引 (0=A, 1=B, 2=C)
 * \return          电流值 (mA), 带符号
 */
static inline float adc_to_current_ma(uint16_t adc_val, uint8_t channel)
{
    int32_t delta = (int32_t)adc_val - (int32_t)adc_offset[channel];
    return (float)delta * scale_ma_per_lsb;
}

/* Public API ----------------------------------------------------------------*/

/**
 * \brief           初始化电流采样模块
 */
void curr_sense_init(void)
{
    new_data_flag = 0;
    curr_filtered[0] = 0.0f;
    curr_filtered[1] = 0.0f;
    curr_filtered[2] = 0.0f;
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/**
 * \brief           查询新数据标志
 */
uint8_t curr_sense_data_ready(void)
{
    return new_data_flag;
}

/**
 * \brief           仅清除 data_ready 标志 (不读电流)
 */
void new_data_flag_clear(void)
{
    new_data_flag = 0;
}

/**
 * \brief           获取 A 相电流 (mA)
 */
float curr_sense_get_ia(void)
{
    uint16_t val;
    __disable_irq();
    val = adc_curr_buf[0];
    __enable_irq();
    float raw = adc_to_current_ma(val, 0); 
    curr_filtered[0] += filter_alpha * (raw - curr_filtered[0]);
    return curr_filtered[0];
}

/**
 * \brief           获取 B 相电流 (mA)
 */
float curr_sense_get_ib(void)
{
    uint16_t val;
    __disable_irq();
    val = adc_curr_buf[1];
    __enable_irq();
    float raw = adc_to_current_ma(val, 1);
    curr_filtered[1] += filter_alpha * (raw - curr_filtered[1]);
    return -curr_filtered[1];
}

/**
 * \brief           获取 C 相电流 (mA)
 */
float curr_sense_get_ic(void)
{
    uint16_t val;
    __disable_irq();
    val = adc_curr_buf[2];
    __enable_irq();
    float raw = adc_to_current_ma(val, 2);
    curr_filtered[2] += filter_alpha * (raw - curr_filtered[2]);
    return -curr_filtered[2];
}

/**
 * \brief           原子读取三相电流, 同时清除 data_ready 标志
 */
void curr_sense_get_all(float *ia, float *ib, float *ic)
{
    uint16_t buf_a, buf_b, buf_c;

    /* 关中断原子拷贝缓冲, 防止 ISR 写入期间被 CPU 读撕裂 */
    __disable_irq();
    buf_a = adc_curr_buf[0];
    buf_b = adc_curr_buf[1];
    buf_c = adc_curr_buf[2];
    new_data_flag = 0;
    __enable_irq();

    /* 原始 ADC -> 电流转换  */
    float raw_a =  adc_to_current_ma(buf_a, 0);
    float raw_b =  adc_to_current_ma(buf_b, 1);
    float raw_c =  adc_to_current_ma(buf_c, 2);

    /* EMA 低通滤波: y[n] = y[n-1] + alpha * (x[n] - y[n-1]) */
    curr_filtered[0] += filter_alpha * (raw_a - curr_filtered[0]);
    curr_filtered[1] += filter_alpha * (raw_b - curr_filtered[1]);
    curr_filtered[2] += filter_alpha * (raw_c - curr_filtered[2]);

    *ia =  curr_filtered[0];
    *ib = -curr_filtered[1];
    *ic = -curr_filtered[2];
}

/**
 * \brief           零点校准
 * \note            需确保电机无电流 (PWM 关闭或电机断开) 再调用.
 *                  采集 num_samples 次后取平均, 更新内部偏移量.
 */
void curr_sense_calibrate(uint32_t num_samples)
{
    uint32_t sum[3] = {0, 0, 0};
    uint32_t i;

    if (num_samples == 0) {
        return;
    }

    /* 采集 num_samples 次, 每通道累加原始 ADC 值 */
    for (i = 0; i < num_samples; i++) {
        /* 等待新的 ADC 扫描完成 (带超时保护, 防 ADC 链断裂死锁) */
        uint32_t timeout = 5000000U;
        while (!new_data_flag && --timeout > 0U) {
            /* 等待注入组 JEOC 中断 */
        }
        if (timeout == 0U) {
            /* ADC 触发链可能断裂, 保持现有偏移量, 放弃本次校准 */
            return;
        }
        __disable_irq();
        sum[0] += adc_curr_buf[0];
        sum[1] += adc_curr_buf[1];
        sum[2] += adc_curr_buf[2];
        new_data_flag = 0;
        __enable_irq();
    }

    /* 平均值写入偏移量表 */
    adc_offset[0] = (uint16_t)(sum[0] / num_samples);
    adc_offset[1] = (uint16_t)(sum[1] / num_samples);
    adc_offset[2] = (uint16_t)(sum[2] / num_samples);
}

/**
 * \brief           设置 EMA 滤波系数
 * \param[in]       alpha  0.0~1.0, 越小滤波越强
 */
void curr_sense_set_filter_alpha(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    filter_alpha = alpha;
}

/**
 * \brief           读取当前零点偏移量
 */
void curr_sense_get_offsets(uint16_t *offset_a, uint16_t *offset_b, uint16_t *offset_c)
{
    *offset_a = adc_offset[0];
    *offset_b = adc_offset[1];
    *offset_c = adc_offset[2];
}

/**
 * \brief           读取原始 ADC 值 (调试用, 不做任何处理)
 */
void curr_sense_get_raw_adc(uint16_t *raw_a, uint16_t *raw_b, uint16_t *raw_c)
{
    __disable_irq();
    *raw_a = adc_curr_buf[0];
    *raw_b = adc_curr_buf[1];
    *raw_c = adc_curr_buf[2];
    __enable_irq();
}

/* HAL Callbacks -------------------------------------------------------------*/

/**
 * \brief           ADC 注入组转换完成回调 (覆盖 HAL 弱定义)
 * \note            由 ADC1_2_IRQHandler -> HAL_ADC_IRQHandler 调用.
 *                  每次 3 通道注入扫描完成后触发.
 *                  从 JDRx 寄存器直接读取, 无 DMA 延迟.
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_curr_buf[0] = hadc->Instance->JDR1;
        adc_curr_buf[1] = hadc->Instance->JDR2;
        adc_curr_buf[2] = hadc->Instance->JDR3;
        new_data_flag = 1;
    }
}
