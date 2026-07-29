/**
 * @file    app_foc.h
 * @brief   FOC 电流环 (20kHz, ADC JEOC 中断内运行)
 * @note    Clarke → Park → PI(Id=0) → 逆Park → SVPWM
 *         编码器 offset 从开环校准读取.
 *         不含速度环、齿槽补偿、故障保护.
 */

#ifndef __APP_FOC_H
#define __APP_FOC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 默认 PI 参数 ---- */

#define FOC_DEFAULT_KP   0.01f     /**< 电流环 Kp (归一化 V/A) */
#define FOC_DEFAULT_KI   3.0f    /**< 电流环 Ki (归一化 V/A/s) */
#define FOC_IQ_MAX       3.0f     /**< Iq 限幅 (A)                 */
#define FOC_V_MAX        0.95f    /**< 电压输出限幅 (归一化)       */

/* ---- 位置环 ---- */

#define FOC_POS_DEFAULT_KP  3.0f    /**< 位置环 Kp (A/deg)              */
#define FOC_POS_DEFAULT_KD  10.0f   /**< 位置环 Kd (A/deg_diff)         */
#define FOC_POS_LOOP_DIV    100     /**< 位置环降频: 20kHz/100=200Hz   */

/* ---- 编码器 ---- */

#define FOC_POLE_PAIRS            7   /**< 极对数                       */
#define FOC_ENCODER_OFFSET_DEG    196  /**< 编码器安装偏移 (电角度, deg) */

/* ---- API ---- */

void foc_init(void);
void foc_start(void);
void foc_stop(void);
uint8_t foc_is_enabled(void);

void foc_set_iq_ref(float iq_amps);
float foc_get_iq_ref(void);

void foc_set_current_kp(float kp);
void foc_set_current_ki(float ki);
void foc_get_current_pi(float *kp, float *ki);

/* 固定电角度测试: foc_test_theta(90) → θ锁定90°, -1 → 退出 */
void foc_test_theta(float deg);

/* 位置环: pos 模式时, foc_isr 内 1kHz 跑位置P → iq_ref */
void foc_pos_enable(uint8_t en);
void foc_set_target(float mech_deg);
float foc_get_position(void);
void foc_set_pos_kp(float kp);
void foc_set_pos_kd(float kd);
void foc_set_pos_ki(float ki);

/* ADC ISR 回调: 读电流 → FOC → 写 PWM (20kHz, ISR 上下文) */
void foc_isr(void);

/* Trace (ISR 捕获, 主循环打印) */
uint8_t foc_trace_ready(void);
void foc_trace_get(int *iq_ref, int *iq, int *id,
                   int *vd, int *vq, int *theta,
                   int *ia, int *ib, int *pos);
void foc_trace_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FOC_H */
