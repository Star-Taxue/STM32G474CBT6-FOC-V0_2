#ifndef __APP_CURRENT_SENSE_H
#define __APP_CURRENT_SENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_config.h"

/**
 * \brief           三相电流零点校准值 (校准参考值, 需根据实际硬件重新校准)
 */
#define CURR_SENSE_OFFSET_A  2041   /* Ia: 实测 ~2040.5 */
#define CURR_SENSE_OFFSET_B  2024   /* Ib: 实测 ~2024.4 */
#define CURR_SENSE_OFFSET_C  1990   /* Ic: 实测 ~1989.7 */

/**
 * \brief           初始化电流采样模块
 * \note            启动 ADC1 注入组 + JEOC 中断 (由 HRTIM ADC_TRG2 触发).
 *                  必须在 MX_ADC1_Init() 和 MX_HRTIM1_Init() 之后调用.
 */
void curr_sense_init(void);

/**
 * \brief           查询是否有新 ADC 数据就绪
 * \retval          1 有新数据, 0 无新数据
 * \note            标志由 HAL_ADC_ConvCpltCallback 置位,
 *                  curr_sense_get_all() 读取后自动清除.
 */
uint8_t curr_sense_data_ready(void);

/**
 * \brief           仅清除 data_ready 标志 (不读电流)
 * \note            用于 openloop_step() 等不需要电流数据但要按 20kHz 同步的场景.
 */
void new_data_flag_clear(void);

/**
 * \brief           获取 A 相电流
 * \return          A 相电流 (mA), 正值 = 电机驱动方向
 */
float curr_sense_get_ia(void);

/**
 * \brief           获取 B 相电流
 * \return          B 相电流 (mA), 正值 = 电机驱动方向
 */
float curr_sense_get_ib(void);

/**
 * \brief           获取 C 相电流
 * \return          C 相电流 (mA), 正值 = 电机驱动方向
 */
float curr_sense_get_ic(void);

/**
 * \brief           原子读取三相电流 (mA)
 * \param[out]      ia   A 相电流 (mA)
 * \param[out]      ib   B 相电流 (mA)
 * \param[out]      ic   C 相电流 (mA)
 * \note            关中断拷贝 DMA 缓冲区, 防止半字撕裂.
 *                  读取后自动清除 data_ready 标志.
 */
void curr_sense_get_all(float *ia, float *ib, float *ic);

/**
 * \brief           零点校准 (电机静止时调用)
 * \param[in]       num_samples  每通道采样次数 (建议 64/128/256)
 * \note            采样期间需确保电机无电流 (无 PWM 输出或电机断开).
 *                  结果更新内部偏移量, 后续 curr_sense_get_* 使用新偏移.
 */
void curr_sense_calibrate(uint32_t num_samples);

/**
 * \brief           读取当前零点偏移量 (调试用)
 * \param[out]      offset_a  A 相偏移 (ADC 原始值)
 * \param[out]      offset_b  B 相偏移 (ADC 原始值)
 * \param[out]      offset_c  C 相偏移 (ADC 原始值)
 */
void curr_sense_get_offsets(uint16_t *offset_a, uint16_t *offset_b, uint16_t *offset_c);

/**
 * \brief           设置 EMA 低通滤波系数
 * \param[in]       alpha  滤波系数 (0.0 ~ 1.0)
 * \note            alpha=1.0 无滤波 (直通), alpha 越小滤波越强.
 *                  默认 = 0.15, 对应 20kHz 采样率下约 500Hz 截止频率.
 *                  公式: y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 */
void curr_sense_set_filter_alpha(float alpha);

/**
 * \brief           读取原始 ADC 三通道值 (调试用)
 * \param[out]      raw_a  CH1 原始 ADC 值 (PA0, Ia)
 * \param[out]      raw_b  CH2 原始 ADC 值 (PA1, Ib)
 * \param[out]      raw_c  CH3 原始 ADC 值 (PA2, Ic)
 * \note            直接拷贝 DMA 缓冲区, 不做偏移校准和滤波.
 *                  用于诊断 ADC/DMA 硬件问题.
 */
void curr_sense_get_raw_adc(uint16_t *raw_a, uint16_t *raw_b, uint16_t *raw_c);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CURRENT_SENSE_H */
