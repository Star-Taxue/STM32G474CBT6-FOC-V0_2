/**
 * @file    app_foc.h
 * @brief   FOC 三环级联控制 (20kHz, ADC JEOS 中断内运行)
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

#define FOC_DEFAULT_KP   0.22f     /**< 电流环 Kp (归一化 V/A), 相值 R=5.12Ω L=0.948mH @400Hz, Vdc=24V */
#define FOC_DEFAULT_KI   1193.0f   /**< 电流环 Ki (归一化 V/A/s) */
#define FOC_IQ_MAX       1.0f     /**< Iq 限幅 (A)                 */
#define FOC_V_MAX        0.95f    /**< 电压输出限幅 (归一化)       */

/* ---- 位置环 ---- */

#define FOC_POS_DEFAULT_KP  1.0f    /**< 位置环 Kp (A/deg)              */
#define FOC_POS_DEFAULT_KD  2.0f    /**< 位置环 Kd (A/deg_diff)         */
#define FOC_POS_LOOP_DIV    100     /**< 位置环降频: 20kHz/100=200Hz   */
#define FOC_POS_DEFAULT_VMAX 360.0f /**< 位置轨迹最大速度 (deg/s, 0=无限) */

/* ---- 速度环 ---- */

#define FOC_SPEED_DEFAULT_KP     0.004f  /**< 速度环 Kp (A/RPM)              */
#define FOC_SPEED_DEFAULT_KI     0.02f   /**< 速度环 Ki (A/RPM/s)           */
#define FOC_SPEED_DEFAULT_RAMP   1000.0f /**< 速度参考斜坡 (RPM/s, 0=无限)   */
#define FOC_SPEED_LOOP_DIV       20      /**< 速度环降频: 20kHz/20=1kHz     */
#define FOC_SPEED_WIN_SLOW       200     /**< 低速估算窗口 (ISR 数, 200=10ms) */
#define FOC_SPEED_SLOW_WIN_RPM   0.0f   /**< 低于此值使用长窗口 (0 = 禁用长窗口, 低速长窗口会造成极限环) */
#define FOC_SPEED_FILTER_ALPHA   0.1f    /**< 速度估算 EMA 滤波系数 (0~1)   */
#define FOC_SPEED_MAX_RPM        2000.0f /**< 速度限幅 (机械 RPM)           */

/* ---- 编码器 ---- */

#define FOC_POLE_PAIRS            11  /**< 极对数 (22 极 → 11 极对, 实测) */
#define FOC_ENCODER_OFFSET_DEG    120  /**< 编码器安装偏移 (电角度, deg) – 11极对, 由 210-90 与实测 110~125 区间确定 */

/* ---- 齿槽/纹波补偿 (整圈位置前馈) ---- */

#define COG_LUT_SIZE         180     /**< 前馈 LUT 点数 (覆盖整圈机械角)   */
#define COG_DEFAULT_GAIN     1.0f    /**< 默认前馈增益 (LUT 为 0 时无作用)  */

/* ---- 编码器角度校正 (整圈 LUT) ---- */

#define ENC_CAL_BINS   256    /**< 编码器校正表点数 (覆盖整圈机械角) */

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

void  foc_set_encoder_offset_deg(float deg);
float foc_get_encoder_offset_deg(void);

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
void foc_set_pos_vmax(float deg_s);   /* 位置轨迹最大速度 (deg/s, 0=无限) */
float foc_get_pos_vmax(void);

/* 速度环: speed 模式时, foc_isr 内 1kHz 跑速度PI → iq_ref */
void foc_speed_enable(uint8_t en);
uint8_t foc_speed_is_enabled(void);
void foc_set_speed_ref(float rpm);
float foc_get_speed_ref(void);
float foc_get_speed_est(void);
void foc_set_speed_kp(float kp);
void foc_set_speed_ki(float ki);
void foc_get_speed_pi(float *kp, float *ki);
void foc_set_speed_ramp(float rpm_s);
float foc_get_speed_ramp(void);

/* 齿槽补偿: 按机械角查 LUT, 前馈叠加到 iq_ref.
 *   LUT 覆盖一个齿槽周期 (360/84 deg), 线性插值. 运行时可通过串口上传/调增益. */
void     foc_cog_enable(uint8_t en);
uint8_t  foc_cog_is_enabled(void);
void     foc_cog_set_gain(float gain);
float    foc_cog_get_gain(void);
void     foc_cog_set_point(uint16_t idx, float val);
float    foc_cog_get_point(uint16_t idx);
uint16_t foc_cog_lut_size(void);

/* 编码器角度校正: 采集模式按角度分箱累加偏差, 校正模式对 mech_deg 查表修正.
 *   修正 mech_deg 同时消除测速假纹波与换相误差. 采集期间应降速度环增益. */
void     enc_cal_enable(uint8_t en);
uint8_t  enc_cal_is_enabled(void);
void     enc_cal_reset(void);
int32_t  enc_cal_get_sum(uint16_t idx);
uint32_t enc_cal_get_cnt(uint16_t idx);
void     enc_corr_enable(uint8_t en);
uint8_t  enc_corr_is_enabled(void);
void     enc_corr_set(uint16_t idx, float deg);
float    enc_corr_get(uint16_t idx);

/* ADC JEOS 回调: 读电流 → FOC → 写 PWM (20kHz, ISR 上下文) */
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
