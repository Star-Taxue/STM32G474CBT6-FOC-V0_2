/**
 * @file    app_foc.c
 * @brief   FOC 电流环 — ADC JEOC 中断内运行 (20kHz)
 * @note    Clarke → Park → PI(Id=0, Iq) → 逆Park → SVPWM → HRTIM
 *          ISR 上下文, 无浮点打印, 不做速度环和故障保护.
 */

#include "main.h"
#include "hrtim.h"
#include "app_foc.h"
#include "app_openloop.h"
#include "app_current_sense.h"
#include "mt6701.h"
#include "motor_config.h"
#include "svpwm.h"
#include <math.h>
#include <stdio.h>
#include <math.h>

/* -------------------------------------------------------------------------- */
/*  数学常量                                                                   */
/* -------------------------------------------------------------------------- */

#define PI          3.14159265358979323846f
#define TWO_PI      6.28318530717958647692f
#define INV_SQRT3   0.57735026918962576451f   /* 1/sqrt(3) */
#define SQRT3_2     0.86602540378443864676f   /* sqrt(3)/2 */
#define DEG_TO_RAD  0.01745329251994329577f

/* PI 控制周期 */
#define FOC_TS      0.00005f                  /* 1/20000 = 50us */

/* 编码器 angle_deg 直接存原始值, 不 fmodf (开环 trace 格式兼容) */
#define ENC_RAW_DEG(angle_deg, pp)  ((angle_deg) * (float)(pp))

/* -------------------------------------------------------------------------- */
/*  PI 控制器                                                                  */
/* -------------------------------------------------------------------------- */

typedef struct {
    float kp, ki;
    float integral;
    float out_min, out_max;
} pi_t;

/** 一步 PI 更新: 条件积分 anti-windup, 线性区正向欧拉 */
static inline float pi_step(pi_t *p, float err)
{
    float out = p->kp * err + p->integral;
    if (out > p->out_max) {
        out = p->out_max;
    } else if (out < p->out_min) {
        out = p->out_min;
    } else {
        p->integral += p->ki * err * FOC_TS;
    }
    return out;
}

static inline void pi_reset(pi_t *p)
{
    p->integral = 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  静态状态                                                                   */
/* -------------------------------------------------------------------------- */

/** 全局 trace 开关 (开环 / FOC 共享) */
uint8_t g_trace_enabled = TRACE_ENABLE_DEFAULT;

static pi_t    pi_d, pi_q;              /* Id / Iq PI 控制器               */
static uint8_t enabled;                 /* FOC 使能标志                    */
static float   iq_ref;                  /* Iq 目标 (A)                     */
static float   encoder_offset_rad;      /* 编码器安装偏移 (rad, 电角度)    */
static float   scale_amps;              /* ADC LSB → Amps 转换系数         */

/* ADC 偏移 (运行时校准值, 由 curr_sense_isr_read 填充) */
static uint16_t adc_off[3];

/* ISR 计算中间值 (保留给 trace) */
static float dbg_id, dbg_iq, dbg_vd, dbg_vq;
static float dbg_theta, dbg_ia, dbg_ib;

/* Trace 快照 (ISR 捕获, 主循环读取打印) */
static volatile uint8_t trace_ready;
static int snap_iq_ref, snap_iq, snap_id;
static int snap_vd, snap_vq, snap_theta;
static int snap_ia, snap_ib;

/* -------------------------------------------------------------------------- */
/*  编码器角度                                                                 */
/* -------------------------------------------------------------------------- */

static uint8_t  fixed_theta_enabled;       /**< 固定角度测试模式              */
static float    fixed_theta_deg;           /**< 固定电角度 (deg)              */

/** 读取编码器 → 电角度 (rad), 含 offset */
static inline float foc_read_elec_angle(void)
{
    if (fixed_theta_enabled) {
        float t = fixed_theta_deg * DEG_TO_RAD;
        if (t >= TWO_PI) t -= TWO_PI;
        if (t < 0.0f)    t += TWO_PI;
        return t;
    }
    mt6701_data_t enc = mt6701_read_angle();
    float elec_deg = -ENC_RAW_DEG(enc.angle_deg, FOC_POLE_PAIRS); /* 反转编码器符号 */
    while (elec_deg >= 360.0f) elec_deg -= 360.0f;
    while (elec_deg < 0.0f)   elec_deg += 360.0f;
    float theta = elec_deg * DEG_TO_RAD + encoder_offset_rad;
    if (theta >= TWO_PI) theta -= TWO_PI;
    if (theta < 0.0f)    theta += TWO_PI;
    return theta;
}

/** 切换固定角度测试模式: foc_test_theta(deg) 启用, 传 -1 退出 */
void foc_test_theta(float deg)
{
    if (deg < 0.0f) {
        fixed_theta_enabled = 0;
    } else {
        fixed_theta_enabled = 1;
        fixed_theta_deg = deg;
    }
}

/* -------------------------------------------------------------------------- */
/*  ISR 电流读取 (无滤波, 纯原始→Amps)                                        */
/* -------------------------------------------------------------------------- */

/**
 * \brief           ISR 专用: 从 ADC buffer 读三相电流 (A)
 * \note            不走 EMA 滤波, 不操作 new_data_flag.
 *                  KCL 验证: A 同相, B/C 反相.
 */
static inline void foc_read_currents(float *ia, float *ib)
{
    /* buf[0]=JDR1=PA2=A, buf[1]=JDR2=PA1=B */
    int32_t raw_a = (int32_t)adc_curr_buf[0] - (int32_t)adc_off[0];
    int32_t raw_b = (int32_t)adc_curr_buf[1] - (int32_t)adc_off[1];
    *ia =  (float)raw_a * scale_amps;          /* A 同相 (运放+接地端) */
    *ib = -(float)raw_b * scale_amps;          /* B 反相 (运放+接电机端) */
}

/* -------------------------------------------------------------------------- */
/*  Trace                                                                    */
/* -------------------------------------------------------------------------- */

/** ISR 内更新 trace 快照 (每 100 次调用一次 = 200Hz) */
static void foc_trace_capture(void)
{
    static uint16_t div = 0;
    if (++div < TRACE_DIV) return;
    div = 0;

    snap_iq_ref = (int)(iq_ref * 1000.0f);
    snap_iq     = (int)(dbg_iq * 1000.0f);
    snap_id     = (int)(dbg_id * 1000.0f);
    snap_vd     = (int)(dbg_vd * 1000.0f);
    snap_vq     = (int)(dbg_vq * 1000.0f);
    snap_theta  = (int)(dbg_theta * 57.2958f);
    snap_ia     = (int)(dbg_ia * 1000.0f);
    snap_ib     = (int)(dbg_ib * 1000.0f);
    trace_ready = 1;
}

/* -------------------------------------------------------------------------- */
/*  FOC ISR 主函数 (20kHz, ADC JEOC 回调内调用)                                 */
/* -------------------------------------------------------------------------- */

/**
 * \brief           FOC 电流环 ISR
 * \note            调用链: ADC JEOC ISR → foc_isr()
 *                  读电流 → 编码器 → Clarke → Park → PI → 逆Park → SVPWM
 */
void foc_isr(void)
{
    /* 1. 读电流 */
    float ia, ib;
    foc_read_currents(&ia, &ib);
    dbg_ia = ia;
    dbg_ib = ib;

    /* 2. 读编码器 → 电角度 */
    float theta = foc_read_elec_angle();
    dbg_theta = theta;

    /* 3. Clarke 变换 (幅值不变): Iαβ = [ia; (ia+2ib)/√3] */
    float i_alpha = ia;
    float i_beta  = (ia + 2.0f * ib) * INV_SQRT3;

    /* 4. Park 变换: Idq = [cosθ, sinθ; -sinθ, cosθ] * Iαβ */
    float s = sinf(theta);
    float c = cosf(theta);
    float id =  i_alpha * c + i_beta * s;
    float iq = -i_alpha * s + i_beta * c;
    dbg_id = id;
    dbg_iq = iq;

    /* 5. 电流 PI: Id→0 (SPMSM), Iq→iq_ref */
    float vd = pi_step(&pi_d, 0.0f - id);
    float vq = pi_step(&pi_q, iq_ref - iq);

    /* 电压圆限制: Vd²+Vq² ≤ FOC_V_MAX², 超出时等比缩放 + 积分反算 */
    {
        float mag_sq = vd * vd + vq * vq;
        float max_sq = FOC_V_MAX * FOC_V_MAX;
        if (mag_sq > max_sq) {
            float scl = FOC_V_MAX / sqrtf(mag_sq);
            vd *= scl;
            vq *= scl;
            pi_d.integral = vd - pi_d.kp * (0.0f - id);
            pi_q.integral = vq - pi_q.kp * (iq_ref - iq);
        }
    }
    dbg_vd = vd;
    dbg_vq = vq;

    /* 6. 逆 Park: Vαβ = [cosθ, -sinθ; sinθ, cosθ] * Vdq */
    float v_alpha = vd * c - vq * s;
    float v_beta  = vd * s + vq * c;

    /* 7. 逆 Clarke → 三相电压 (归一化) */
    float va =  v_alpha;
    float vb = -0.5f * v_alpha + SQRT3_2 * v_beta;
    float vc = -0.5f * v_alpha - SQRT3_2 * v_beta;

    /* 8. SVPWM + 写 HRTIM CMP 寄存器 */
    svpwm_write(va, vb, vc, 1);

    /* 9. Trace 捕获 (200Hz 降频) */
    foc_trace_capture();
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                 */
/* -------------------------------------------------------------------------- */

void foc_init(void)
{
    enabled = 0;
    iq_ref  = 0.0f;

    /* 编码器偏移, 独立于开环 */
    encoder_offset_rad = (float)FOC_ENCODER_OFFSET_DEG * DEG_TO_RAD;

    /* 初始化电流 PI */
    pi_d.kp = FOC_DEFAULT_KP;
    pi_d.ki = FOC_DEFAULT_KI;
    pi_d.out_min = -FOC_V_MAX;
    pi_d.out_max =  FOC_V_MAX;
    pi_reset(&pi_d);

    pi_q.kp = FOC_DEFAULT_KP;
    pi_q.ki = FOC_DEFAULT_KI;
    pi_q.out_min = -FOC_V_MAX;
    pi_q.out_max =  FOC_V_MAX;
    pi_reset(&pi_q);

    /* ADC 比例因子: A/LSB */
    scale_amps = CURR_SENSE_ADC_VREF
               / (CURR_SENSE_ADC_RESOLUTION * CURR_SENSE_OPAMP_GAIN
                  * CURR_SENSE_SHUNT_RESISTANCE);

    /* 读取运行时校准的 ADC offset */
    curr_sense_get_offsets(&adc_off[0], &adc_off[1], &adc_off[2]);
}

void foc_start(void)
{
    if (enabled) return;

    /* 编码器偏移, 独立于开环 */
    encoder_offset_rad = (float)FOC_ENCODER_OFFSET_DEG * DEG_TO_RAD;

    /* 重置 PI */
    pi_reset(&pi_d);
    pi_reset(&pi_q);
    iq_ref = 0.0f;

    /* 重读 ADC offset (运行时校准值) */
    curr_sense_get_offsets(&adc_off[0], &adc_off[1], &adc_off[2]);

    enabled = 1;
}

void foc_stop(void)
{
    enabled = 0;
    iq_ref  = 0.0f;
    pi_reset(&pi_d);
    pi_reset(&pi_q);

    /* 写零电压 */
    svpwm_write(0.0f, 0.0f, 0.0f, 1);
}

uint8_t foc_is_enabled(void)
{
    return enabled;
}

/* ---- Iq 参考 ---- */

void foc_set_iq_ref(float iq_amps)
{
    if (iq_amps >  FOC_IQ_MAX) iq_amps =  FOC_IQ_MAX;
    if (iq_amps < -FOC_IQ_MAX) iq_amps = -FOC_IQ_MAX;
    iq_ref = iq_amps;
}

float foc_get_iq_ref(void)
{
    return iq_ref;
}

/* ---- PI 调参 ---- */

void foc_set_current_kp(float kp) { pi_d.kp = kp; pi_q.kp = kp; }
void foc_set_current_ki(float ki) { pi_d.ki = ki; pi_q.ki = ki; }

void foc_get_current_pi(float *kp, float *ki)
{
    *kp = pi_d.kp;
    *ki = pi_d.ki;
}

/* ---- Trace ---- */

uint8_t foc_trace_ready(void)
{
    return trace_ready;
}

void foc_trace_get(int *iq_ref_out, int *iq_out, int *id_out,
                   int *vd_out, int *vq_out, int *theta_out,
                   int *ia_out, int *ib_out)
{
    __disable_irq();
    *iq_ref_out = snap_iq_ref;
    *iq_out     = snap_iq;
    *id_out     = snap_id;
    *vd_out     = snap_vd;
    *vq_out     = snap_vq;
    *theta_out  = snap_theta;
    *ia_out     = snap_ia;
    *ib_out     = snap_ib;
    trace_ready = 0;
    __enable_irq();
}

/**
 * \brief           FOC trace 打印 (主循环调用)
 */
void foc_trace_poll(void)
{
    if (!trace_ready) return;

    int iqr, iq, id, vd, vq, th, ia, ib;
    foc_trace_get(&iqr, &iq, &id, &vd, &vq, &th, &ia, &ib);
    printf("F:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
           iqr, iq, id, vd, vq, th, ia, ib);
}
