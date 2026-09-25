/**
 * @file    app_foc.h
 * @brief   FOC 三环级联控制 (20kHz, ADC JEOC 中断内运行)
 * @note    Clarke → Park → PI(Id=0) → 逆Park → SVPWM
 *         级联: 位置PD(200Hz) → 速度PI(1kHz) → 电流PI(20kHz)
 *         编码器 offset 从开环校准读取.
 */

#ifndef __APP_FOC_H
#define __APP_FOC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 默认 PI 参数 ---- */

#define FOC_DEFAULT_KP   0.01f     /**< 电流环 Kp (归一化 V/A) */
#define FOC_DEFAULT_KI   5.0f    /**< 电流环 Ki (归一化 V/A/s) */
#define FOC_IQ_MAX       10.0f     /**< Iq 限幅 (A)                 */
#define FOC_V_MAX        0.95f    /**< 电压输出限幅 (归一化)       */

/* ---- 位置环 ---- */

#define FOC_POS_DEFAULT_KP  0.05f    /**< 位置环 Kp (A/deg)              */
#define FOC_POS_DEFAULT_KD  1.5f   /**< 位置环 Kd (A/deg_diff)         */
#define FOC_POS_LOOP_DIV    100     /**< 位置环降频: 20kHz/100=200Hz   */

/* ---- 速度环 ---- */

#define FOC_SPEED_DEFAULT_KP     0.25f   /**< 速度环 Kp (A/RPM)              */
#define FOC_SPEED_DEFAULT_KI     0.5f    /**< 速度环 Ki (A/RPM/s)           */
#define FOC_SPEED_LOOP_DIV       20      /**< 速度环降频: 20kHz/20=1kHz     */
#define FOC_SPEED_WIN_SLOW       200     /**< 低速估算窗口 (ISR 数, 200=10ms) */
#define FOC_SPEED_SLOW_WIN_RPM   60.0f   /**< 低于此转速切换长窗口 (RPM)   */
#define FOC_SPEED_FILTER_ALPHA   0.1f    /**< 速度估算 EMA 滤波系数 (0~1)   */
#define FOC_SPEED_MAX_RPM        10000.0f /**< 速度限幅 (机械 RPM)           */
#define FOC_FRICTION_FF_AMP      0.4f     /**< 摩擦前馈默认电流 (A)          */
#define FOC_FRICTION_FF_DEADZONE 0.5f     /**< 摩擦前馈死区 (RPM)            */
#define FOC_FRICTION_FF_ON_ERR   2.0f     /**< 前馈生效误差阈值 (RPM)        */

/* ---- 齿槽转矩前馈 (查表) ---- */

#define COG_LUT_SIZE            84      /**< 齿槽表槽数 (12N14P: LCM=84 槽/电周期) */
#define COG_CAL_DEFAULT_RPM     60.0f   /**< 默认标定转速 (机械 RPM)        */
#define COG_CAL_DEFAULT_REVS    10      /**< 默认标定圈数 (机械圈)          */
#define COG_CAL_SETTLE_RPM      5.0f    /**< 转速稳定判据 (RPM)             */
#define COG_LUT_MAX_AMP         2.0f    /**< 表幅值限幅 (A)                 */

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

/* 位置环: pos 模式时, foc_isr 内 200Hz 跑位置PD
 *   速度环使能: 位置PD → speed_ref → 速度PI → iq_ref
 *   速度环关闭: 位置PD → iq_ref (直连)                        */
void foc_pos_enable(uint8_t en);
void foc_set_target(float mech_deg);
float foc_get_position(void);
void foc_set_pos_kp(float kp);
void foc_set_pos_kd(float kd);
void foc_set_pos_ki(float ki);

/* 速度环: speed 模式时, foc_isr 内 1kHz 跑速度PI → iq_ref */
void foc_speed_enable(uint8_t en);
uint8_t foc_speed_is_enabled(void);
void foc_set_speed_ref(float rpm);
float foc_get_speed_ref(void);
float foc_get_speed_est(void);
void foc_set_speed_kp(float kp);
void foc_set_speed_ki(float ki);
void foc_set_friction_ff(float amps);
float foc_get_friction_ff(void);

/* ---- 齿槽前馈 ---- */

void cog_calib_start(float rpm, uint32_t revs);
void cog_set_enabled(uint8_t en);
uint8_t cog_is_enabled(void);
void cog_calib_notify(void);
void cog_print_status(void);
void foc_get_speed_pi(float *kp, float *ki);

/* ADC ISR 回调: 读电流 → FOC → 写 PWM (20kHz, ISR 上下文) */
void foc_isr(void);

/* Trace (ISR 捕获, 主循环打印) */
uint8_t foc_trace_ready(void);
void foc_trace_get(int *iq_ref, int *iq, int *id,
                   int *vd, int *vq, int *theta,
                   int *ia, int *ib, int *pos,
                   int *speed_ref, int *speed_est);
void foc_trace_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FOC_H */
