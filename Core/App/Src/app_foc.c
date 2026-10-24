/**
 * @file    app_foc.c
 * @brief   FOC 三环级联 — ADC JEOS 中断内运行 (20kHz)
 * @note    级联: 位置PD(200Hz) → 速度PI(1kHz) → 电流PI(20kHz)
 *          Clarke → Park → PI(Id=0, Iq) → 逆Park → SVPWM → HRTIM
 *          ISR 上下文, 无浮点打印.
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

/* 编码器 angle_deg 直接存原始值 */

/* -------------------------------------------------------------------------- */
/*  PI 控制器                                                                  */
/* -------------------------------------------------------------------------- */

typedef struct {
    float kp, ki;
    float integral;
    float out_min, out_max;
} pi_t;

/** 一步 PI 更新: 条件积分 anti-windup, 线性区正向欧拉 */
static inline float pi_step(pi_t *p, float err, float dt)
{
    float out = p->kp * err + p->integral;
    if (out > p->out_max) {
        out = p->out_max;
    } else if (out < p->out_min) {
        out = p->out_min;
    } else {
        p->integral += p->ki * err * dt;
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

/* 位置环 */
static uint8_t  pos_mode;               /* 位置模式使能                    */
static float    pos_target_deg;         /* 目标机械角 (deg)                */
static float    pos_kp;                 /* 位置 P 增益 (A/deg_mech)        */
static float    pos_kd;                 /* 位置 D 增益 (A/(deg/s))         */
static float    pos_ki;                 /* 位置 I 增益 (A/(deg*s))         */
static float    pos_integral;           /* 位置积分器                      */
static uint8_t  pos_div;                /* 降频计数: 0..POS_LOOP_DIV-1     */
static float    pos_current;            /* 当前位置 (机械角 deg)            */
static float    pos_prev;               /* 上次位置 (deg), 差分用           */
static float    pos_traj_deg;           /* 位置轨迹 (deg)                  */
static float    pos_traj_vel;           /* 位置轨迹速度 (deg/s)            */
static float    pos_vmax;               /* 位置轨迹最大速度 (deg/s, 0=无限) */

/* 速度环 */
static uint8_t  speed_mode;             /* 速度模式使能                    */
static float    speed_ref;              /* 目标速度 (机械 RPM)              */
static float    speed_est;              /* 估算速度 (机械 RPM, EMA滤波)    */
static float    speed_kp;               /* 速度 P 增益 (A/RPM)             */
static float    speed_ki;               /* 速度 I 增益 (A/RPM/s)           */
static float    speed_ramp;             /* 速度参考斜坡 (RPM/s, 0=无限)    */
static float    speed_ref_ramped;       /* 斜坡后的速度参考                */
static float    speed_integral;         /* 速度积分器                      */
static uint8_t  speed_div;              /* 降频计数: 0..SPEED_LOOP_DIV-1   */
static uint16_t est_div;                /* 估算窗口降频计数                */
static float    speed_pos_accum;        /* 位置增量累计 (deg), 速度估算用  */
static float    speed_pos_prev;         /* 上次机械角 (deg), 速度差分用    */
static float    speed_integral_max;     /* 积分上限 (A)                    */
static float    speed_out_max;          /* 输出上限 (A)                    */

#define FOC_SPEED_TS  (FOC_TS * FOC_SPEED_LOOP_DIV)  /* 速度环周期 1ms */

/* 齿槽补偿 (位置前馈) */
static uint8_t cog_enabled;             /* 前馈使能                      */
static float   cog_gain;                /* 前馈增益                      */
static float   cog_lut[COG_LUT_SIZE];   /* 前馈 LUT (整圈机械角)         */
static float   cog_ff;                  /* 上次前馈量 (A), 调试用        */

/* 编码器角度校正 */
static uint8_t  enc_cal_on;                       /* 采集模式使能         */
static int32_t  enc_cal_sum[ENC_CAL_BINS];        /* 偏差累加 (millideg)  */
static uint32_t enc_cal_cnt[ENC_CAL_BINS];        /* 每箱样本数           */
static int16_t  enc_corr_lut[ENC_CAL_BINS];       /* 校正表 (millideg)    */
static uint8_t  enc_corr_on;                      /* 校正使能             */

/* ADC 偏移 (运行时校准值, 由 curr_sense_isr_read 填充) */
static uint16_t adc_off[3];

/* ISR 计算中间值 (保留给 trace) */
static float dbg_id, dbg_iq, dbg_vd, dbg_vq;
static float dbg_theta, dbg_ia, dbg_ib;

/* 电流反馈低通状态 */
static float curr_lpf_a, curr_lpf_b;

/* Trace 快照 (ISR 捕获, 主循环读取打印) */
static volatile uint8_t trace_ready;
static int snap_iq_ref, snap_iq, snap_id;
static int snap_vd, snap_vq, snap_theta;
static int snap_ia, snap_ib, snap_pos;
static int snap_speed_ref, snap_speed_est;

/* -------------------------------------------------------------------------- */
/*  编码器角度                                                                 */
/* -------------------------------------------------------------------------- */

static uint8_t  fixed_theta_enabled;       /**< 固定角度测试模式              */
static float    fixed_theta_deg;           /**< 固定电角度 (deg)              */

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
/*  ISR 电流读取 (一阶 IIR 低通 → Amps)                                        */
/* -------------------------------------------------------------------------- */

/**
 * \brief           ISR 专用: 从 ADC buffer 读三相电流 (A)
 * \note            一阶 IIR 低通抑制开关噪声 (裸采样噪声会被 PI 直接放大).
 *                  不操作 new_data_flag. KCL 验证: A 同相, B/C 反相.
 */
static inline void foc_read_currents(float *ia, float *ib)
{
    /* buf[0]=JDR1=PA2=A, buf[1]=JDR2=PA1=B */
    int32_t raw_a = (int32_t)adc_curr_buf[0] - (int32_t)adc_off[0];
    int32_t raw_b = (int32_t)adc_curr_buf[1] - (int32_t)adc_off[1];
    float ra =  (float)raw_a * scale_amps;     /* A 同相 (运放+接地端) */
    float rb = -(float)raw_b * scale_amps;     /* B 反相 (运放+接电机端) */

    curr_lpf_a += FOC_CURR_LPF_ALPHA * (ra - curr_lpf_a);
    curr_lpf_b += FOC_CURR_LPF_ALPHA * (rb - curr_lpf_b);
    *ia = curr_lpf_a;
    *ib = curr_lpf_b;
}

/* -------------------------------------------------------------------------- */
/*  Trace                                                                    */
/* -------------------------------------------------------------------------- */

/** ISR 内更新 trace 快照 (每 100 次调用一次 = 200Hz) */
static void foc_trace_capture(void)
{
    static uint16_t div = 0;
    if (g_trace_enabled == 0U) {
        div = 0U;
        return;
    }
    if (++div < TRACE_DIV) return;
    div = 0;

#if TRACE_SEND_IQ_REF
    snap_iq_ref = (int)(iq_ref * 1000.0f);
#else
    snap_iq_ref = 0;
#endif
#if TRACE_SEND_IQ
    snap_iq = (int)(dbg_iq * 1000.0f);
#else
    snap_iq = 0;
#endif
#if TRACE_SEND_ID
    snap_id = (int)(dbg_id * 1000.0f);
#else
    snap_id = 0;
#endif
#if TRACE_SEND_VD
    snap_vd = (int)(dbg_vd * 1000.0f);
#else
    snap_vd = 0;
#endif
#if TRACE_SEND_VQ
    snap_vq = (int)(dbg_vq * 1000.0f);
#else
    snap_vq = 0;
#endif
#if TRACE_SEND_THETA
    snap_theta = (int)(dbg_theta * 57.2958f);
#else
    snap_theta = 0;
#endif
#if TRACE_SEND_IA
    snap_ia = (int)(dbg_ia * 1000.0f);
#else
    snap_ia = 0;
#endif
#if TRACE_SEND_IB
    snap_ib = (int)(dbg_ib * 1000.0f);
#else
    snap_ib = 0;
#endif
#if TRACE_SEND_POS
    snap_pos = (int)(pos_current * 10.0f);
#else
    snap_pos = 0;
#endif
#if TRACE_SEND_SPEED_REF
    snap_speed_ref = (int)(speed_ref);
#else
    snap_speed_ref = 0;
#endif
#if TRACE_SEND_SPEED_EST
    snap_speed_est = (int)(speed_est);
#else
    snap_speed_est = 0;
#endif
    trace_ready    = 1;
}

/* -------------------------------------------------------------------------- */
/*  编码器角度校正                                                              */
/* -------------------------------------------------------------------------- */

/**
 * \brief           编码器角度处理: 采集累加 + 校正查表
 * \param[in]       raw  编码器原始机械角 (deg, [0,360))
 * \retval          校正后机械角 (deg); 未使能校正时原样返回
 * \note            采集: 用原始角分箱累加偏差(箱中心为参考);
 *                  校正: 对原始角查表并线性插值, 减去偏差.
 */
static inline float enc_process(float raw)
{
    if (enc_cal_on) {
        uint16_t b = (uint16_t)(raw * ((float)ENC_CAL_BINS / 360.0f));
        if (b >= ENC_CAL_BINS) b = ENC_CAL_BINS - 1;
        float c = ((float)b + 0.5f) * (360.0f / (float)ENC_CAL_BINS);
        enc_cal_sum[b] += (int32_t)((raw - c) * 1000.0f);
        enc_cal_cnt[b]++;
    }
    if (enc_corr_on) {
        float p = raw * ((float)ENC_CAL_BINS / 360.0f);
        if (p < 0.0f) p = 0.0f;
        uint16_t b0 = (uint16_t)p;
        if (b0 >= ENC_CAL_BINS) b0 = ENC_CAL_BINS - 1;
        float fr = p - (float)b0;
        uint16_t b1 = (uint16_t)((b0 + 1U) % ENC_CAL_BINS);
        float corr = (float)enc_corr_lut[b0]
                   + fr * (float)(enc_corr_lut[b1] - enc_corr_lut[b0]);
        return raw - corr * 0.001f;
    }
    return raw;
}

/* -------------------------------------------------------------------------- */
/*  FOC ISR 主函数 (20kHz, ADC JEOS 回调内调用)                                 */
/* -------------------------------------------------------------------------- */

/**
 * \brief           FOC 三环级联 ISR
 * \note            调用链: ADC JEOS ISR → foc_isr()
 *                  20kHz: 电流PI
 *                   1kHz: 速度PI (级联: 位置PD → speed_ref → 速度PI → iq_ref)
 *                   200Hz: 位置PD
 */
void foc_isr(void)
{
    /* 1. 读电流 */
    float ia, ib;
    foc_read_currents(&ia, &ib);
    dbg_ia = ia;
    dbg_ib = ib;

    /* 2. 读编码器 → 电角度 + 机械角 (一次 SPI 读取) */
    mt6701_data_t enc = mt6701_read_angle();
    float mech_deg = enc_process(enc.angle_deg);
    pos_current = mech_deg;     /* 位置环使用, trace 使用 */
    float elec_deg = -mech_deg * (float)FOC_POLE_PAIRS; /* 反转编码器符号 */
    while (elec_deg >= 360.0f) elec_deg -= 360.0f;
    while (elec_deg < 0.0f)    elec_deg += 360.0f;

    /* 固定角度测试模式 */
    float theta;
    if (fixed_theta_enabled) {
        float t = fixed_theta_deg * DEG_TO_RAD;
        if (t >= TWO_PI) t -= TWO_PI;
        if (t < 0.0f)    t += TWO_PI;
        theta = t;
    } else {
        theta = elec_deg * DEG_TO_RAD + encoder_offset_rad;
        if (theta >= TWO_PI) theta -= TWO_PI;
        if (theta < 0.0f)    theta += TWO_PI;
    }
    dbg_theta = theta;

    /* 3. 速度估算: 累计位置增量 (20kHz), 每 1kHz 计算速度 + EMA 滤波 */
    {
        float delta = mech_deg - speed_pos_prev;
        if (delta > 180.0f)       delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        speed_pos_prev = mech_deg;
        speed_pos_accum += delta;
    }

    /* 速度估算: 自适应窗口 (低速 10ms 降量化噪声, 高速 1ms 保动态) */
    {
        uint16_t speed_win = (fabsf(speed_est) < FOC_SPEED_SLOW_WIN_RPM)
                           ? FOC_SPEED_WIN_SLOW : FOC_SPEED_LOOP_DIV;
        if (++est_div >= speed_win) {
            est_div = 0;
            /* raw_rpm = accum_deg / dt / 360 * 60, dt = speed_win * FOC_TS */
            float raw_rpm = speed_pos_accum * (1000.0f / 6.0f)
                          * ((float)FOC_SPEED_LOOP_DIV / (float)speed_win);
            /* EMA 滤波: 按窗口换算 alpha, 保持约 10ms 时间常数 */
            float alpha = 1.0f - powf(1.0f - FOC_SPEED_FILTER_ALPHA,
                                      (float)speed_win / (float)FOC_SPEED_LOOP_DIV);
            speed_est += alpha * (raw_rpm - speed_est);
            speed_pos_accum = 0.0f;
        }
    }

    /* 速度 PI (固定 1kHz, 与估算窗口解耦) */
    if (++speed_div >= FOC_SPEED_LOOP_DIV) {
        speed_div = 0;

        /* 4. 速度 PI (1kHz): 级联时 speed_ref 由位置PD设置, 否则由用户设置 */
        /*    符号约定: 负 Iq → 正转 (与位置环一致: iq_ref = -kp*err)       */
        if (speed_mode) {
            if (speed_ramp > 0.0f) {
                float step = speed_ramp * FOC_SPEED_TS;
                if (speed_ref > speed_ref_ramped + step) {
                    speed_ref_ramped += step;
                } else if (speed_ref < speed_ref_ramped - step) {
                    speed_ref_ramped -= step;
                } else {
                    speed_ref_ramped = speed_ref;
                }
            } else {
                speed_ref_ramped = speed_ref;
            }
            float speed_err = speed_ref_ramped - speed_est;
            /* 条件积分 anti-windup */
            float out = speed_kp * speed_err + speed_integral;
            if (out > speed_out_max) {
                out = speed_out_max;
            } else if (out < -speed_out_max) {
                out = -speed_out_max;
            } else {
                speed_integral += speed_ki * speed_err * FOC_SPEED_TS;
                if (speed_integral >  speed_integral_max)
                    speed_integral =  speed_integral_max;
                if (speed_integral < -speed_integral_max)
                    speed_integral = -speed_integral_max;
            }
            iq_ref = -out;  /* 负号: speed_err>0 (需加速) → 负 Iq → 正转 */

            if (iq_ref >  FOC_IQ_MAX) iq_ref =  FOC_IQ_MAX;
            if (iq_ref < -FOC_IQ_MAX) iq_ref = -FOC_IQ_MAX;
        }
    }

    /* 5. 位置环 PD (200Hz), D用位置差分                                */
    /*    速度环使能: 位置PD → speed_ref; 否则 位置PD → iq_ref (直连)  */
    if (pos_mode) {
        if (++pos_div >= FOC_POS_LOOP_DIV) {
            pos_div = 0;
            /* 位置轨迹: pos_traj_deg 以 pos_vmax 限速逼近 pos_target_deg */
            if (pos_vmax > 0.0f) {
                float pdt = FOC_TS * FOC_POS_LOOP_DIV;
                float dlim = pos_vmax * pdt;
                float dtg = pos_target_deg - pos_traj_deg;
                if (dtg > 180.0f)       dtg -= 360.0f;
                else if (dtg < -180.0f) dtg += 360.0f;
                if (dtg > dlim)       dtg = dlim;
                else if (dtg < -dlim) dtg = -dlim;
                pos_traj_deg += dtg;
                while (pos_traj_deg >= 360.0f) pos_traj_deg -= 360.0f;
                while (pos_traj_deg < 0.0f)    pos_traj_deg += 360.0f;
                pos_traj_vel = dtg / pdt;
            } else {
                pos_traj_deg = pos_target_deg;
                pos_traj_vel = 0.0f;
            }
            float err = pos_traj_deg - mech_deg;
            if (err > 180.0f)       err -= 360.0f;
            else if (err < -180.0f) err += 360.0f;
            float d = mech_deg - pos_prev;
            if (d > 180.0f)       d -= 360.0f;
            else if (d < -180.0f) d += 360.0f;
            pos_prev = mech_deg;
            /* pos_ki 积分项 (默认=0) */
            if (pos_ki > 0.0f) {
                pos_integral += pos_ki * err * (FOC_TS * FOC_POS_LOOP_DIV);
                if (pos_integral >  FOC_IQ_MAX) pos_integral =  FOC_IQ_MAX;
                if (pos_integral < -FOC_IQ_MAX) pos_integral = -FOC_IQ_MAX;
            }
            float pos_out = -pos_kp * err + pos_kd * d + pos_integral;

            if (speed_mode) {
                /* 位置PD 输出 (Iq量纲) → 速度给定 (RPM)
                   符号: pos_out = -kp*err + kd*d (正向误差→负值)
                   翻号后: speed_ref = -pos_out (正向误差→正速度)    */
                speed_ref = -pos_out + pos_traj_vel * (1.0f / 6.0f);  /* 轨迹速度前馈 (deg/s→RPM) */
                if (speed_ref >  FOC_SPEED_MAX_RPM) speed_ref =  FOC_SPEED_MAX_RPM;
                if (speed_ref < -FOC_SPEED_MAX_RPM) speed_ref = -FOC_SPEED_MAX_RPM;
            } else {
                /* 位置PD 输出 → Iq 给定 (直接) */
                if (pos_out >  FOC_IQ_MAX) pos_out =  FOC_IQ_MAX;
                if (pos_out < -FOC_IQ_MAX) pos_out = -FOC_IQ_MAX;
                iq_ref = pos_out;
            }
        }
    }

    /* 5b. 位置前馈: 按整圈机械角查 LUT, 线性插值后叠加到 iq_ref */
    if (cog_enabled) {
        float p = mech_deg * ((float)COG_LUT_SIZE / 360.0f);   /* [0,180) */
        if (p < 0.0f) p = 0.0f;
        uint16_t i0 = (uint16_t)p;
        if (i0 >= COG_LUT_SIZE) i0 = COG_LUT_SIZE - 1;
        float frac = p - (float)i0;
        uint16_t i1 = (uint16_t)((i0 + 1U) % COG_LUT_SIZE);
        float lut = cog_lut[i0] + frac * (cog_lut[i1] - cog_lut[i0]);
        cog_ff = cog_gain * lut;
        iq_ref += cog_ff;
        if (iq_ref >  FOC_IQ_MAX) iq_ref =  FOC_IQ_MAX;
        if (iq_ref < -FOC_IQ_MAX) iq_ref = -FOC_IQ_MAX;
    }

    /* 6. Clarke 变换 (幅值不变): Iαβ = [ia; (ia+2ib)/√3] */
    float i_alpha = ia;
    float i_beta  = (ia + 2.0f * ib) * INV_SQRT3;

    /* 7. Park 变换: Idq = [cosθ, sinθ; -sinθ, cosθ] * Iαβ */
    float s = sinf(theta);
    float c = cosf(theta);
    float id =  i_alpha * c + i_beta * s;
    float iq = -i_alpha * s + i_beta * c;
    dbg_id = id;
    dbg_iq = iq;

    /* 8. 电流 PI: Id→0 (SPMSM), Iq→iq_ref */
    float vd = pi_step(&pi_d, 0.0f - id, FOC_TS);
    float vq = pi_step(&pi_q, iq_ref - iq, FOC_TS);

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

    /* 9. 逆 Park: Vαβ = [cosθ, -sinθ; sinθ, cosθ] * Vdq */
    float v_alpha = vd * c - vq * s;
    float v_beta  = vd * s + vq * c;

    /* 10. 逆 Clarke → 三相电压 (归一化) */
    float va =  v_alpha;
    float vb = -0.5f * v_alpha + SQRT3_2 * v_beta;
    float vc = -0.5f * v_alpha - SQRT3_2 * v_beta;

    /* 11. SVPWM + 写 HRTIM CMP 寄存器 */
    svpwm_write(va, vb, vc, 1);

    /* 12. Trace 捕获 (200Hz 降频) */
    foc_trace_capture();
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                 */
/* -------------------------------------------------------------------------- */

void foc_init(void)
{
    enabled = 0;
    iq_ref  = 0.0f;

    /* 位置环默认值 */
    pos_mode   = 0;
    pos_kp     = FOC_POS_DEFAULT_KP;
    pos_kd     = FOC_POS_DEFAULT_KD;
    pos_ki     = 0.0f;   /* 默认关积分, 需要时 pos_ki 设置 */
    pos_integral = 0.0f;
    pos_div    = 0;
    pos_target_deg = 0.0f;
    pos_prev   = 0.0f;
    pos_traj_deg = 0.0f;
    pos_traj_vel = 0.0f;
    pos_vmax   = FOC_POS_DEFAULT_VMAX;

    /* 速度环默认值 */
    speed_mode     = 0;
    speed_ref      = 0.0f;
    speed_est      = 0.0f;
    speed_kp       = FOC_SPEED_DEFAULT_KP;
    speed_ki       = FOC_SPEED_DEFAULT_KI;
    speed_ramp     = FOC_SPEED_DEFAULT_RAMP;
    speed_ref_ramped = 0.0f;
    speed_integral = 0.0f;
    speed_div      = 0;
    est_div        = 0;
    speed_pos_accum = 0.0f;
    speed_pos_prev  = 0.0f;
    speed_integral_max = FOC_IQ_MAX;
    speed_out_max      = FOC_IQ_MAX;

    curr_lpf_a = 0.0f;
    curr_lpf_b = 0.0f;

    /* 齿槽补偿: 默认关闭, LUT 清零 */
    cog_enabled = 0;
    cog_gain    = COG_DEFAULT_GAIN;
    cog_ff      = 0.0f;
    {
        uint16_t i;
        for (i = 0; i < COG_LUT_SIZE; ++i) {
            cog_lut[i] = 0.0f;
        }
    }

    /* 编码器校正: 默认关闭, 累加器与校正表清零 */
    enc_cal_on  = 0;
    enc_corr_on = 0;
    {
        uint16_t i;
        for (i = 0; i < ENC_CAL_BINS; ++i) {
            enc_cal_sum[i]  = 0;
            enc_cal_cnt[i]  = 0;
            enc_corr_lut[i] = 0;
        }
    }

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

    /* 重置电流 PI */
    pi_reset(&pi_d);
    pi_reset(&pi_q);
    iq_ref = 0.0f;

    /* 重置速度环 */
    speed_integral = 0.0f;
    speed_est      = 0.0f;
    speed_div      = 0;
    speed_pos_accum = 0.0f;
    /* 初始化速度估算位置基准 (避免启动瞬间 delta 尖峰) */
    mt6701_data_t enc = mt6701_read_angle();
    speed_pos_prev = enc.angle_deg;

    /* 重读 ADC offset (运行时校准值) */
    curr_sense_get_offsets(&adc_off[0], &adc_off[1], &adc_off[2]);

    curr_lpf_a = 0.0f;
    curr_lpf_b = 0.0f;

    enabled = 1;
}

void foc_stop(void)
{
    enabled = 0;
    iq_ref  = 0.0f;
    speed_ref = 0.0f;
    speed_integral = 0.0f;
    speed_est = 0.0f;
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

/* ---- 位置环 ---- */

void foc_pos_enable(uint8_t en)
{
    pos_mode = en;
    pos_div  = 0;
    pos_integral = 0.0f;
    if (en) {
        mt6701_data_t enc = mt6701_read_angle();
        pos_prev     = enc.angle_deg;
        pos_traj_deg = enc.angle_deg;   /* 轨迹从当前位置起步 */
    } else {
        /* 关位置环时: 若速度环开着则保持, 否则清零 iq_ref */
        if (!speed_mode) {
            iq_ref = 0.0f;
        }
    }
}

void foc_set_target(float mech_deg)
{
    while (mech_deg >= 360.0f) mech_deg -= 360.0f;
    while (mech_deg < 0.0f)    mech_deg += 360.0f;
    pos_target_deg = mech_deg;
    pos_integral = 0.0f;  /* 换目标时重置积分 */
}

float foc_get_position(void)
{
    return pos_current;
}

void foc_set_pos_kp(float kp) { pos_kp = kp; }
void foc_set_pos_kd(float kd) { pos_kd = kd; }
void foc_set_pos_ki(float ki) { pos_ki = ki; pos_integral = 0.0f; }
void foc_set_pos_vmax(float deg_s) { pos_vmax = deg_s; }
float foc_get_pos_vmax(void) { return pos_vmax; }

/* ---- 速度环 ---- */

void foc_speed_enable(uint8_t en)
{
    speed_mode = en;
    speed_div = 0;
    speed_integral = 0.0f;
    speed_pos_accum = 0.0f;
    if (en) {
        /* 初始化速度估算位置基准 */
        mt6701_data_t enc = mt6701_read_angle();
        speed_pos_prev = enc.angle_deg;
        speed_ref_ramped = speed_est;
    } else {
        speed_ref = 0.0f;
        speed_est = 0.0f;
        /* 关速度环时: 若位置环开着, 由位置PD直接输出 iq_ref */
        if (!pos_mode) {
            iq_ref = 0.0f;
        }
    }
}

uint8_t foc_speed_is_enabled(void)
{
    return speed_mode;
}

void foc_set_speed_ref(float rpm)
{
    if (rpm >  FOC_SPEED_MAX_RPM) rpm =  FOC_SPEED_MAX_RPM;
    if (rpm < -FOC_SPEED_MAX_RPM) rpm = -FOC_SPEED_MAX_RPM;
    speed_ref = rpm;
}

float foc_get_speed_ref(void)
{
    return speed_ref;
}

float foc_get_speed_est(void)
{
    return speed_est;
}

void foc_set_speed_kp(float kp) { speed_kp = kp; }
void foc_set_speed_ki(float ki) { speed_ki = ki; speed_integral = 0.0f; }
void foc_set_speed_ramp(float rpm_s) { speed_ramp = rpm_s; }
float foc_get_speed_ramp(void) { return speed_ramp; }

void foc_get_speed_pi(float *kp, float *ki)
{
    *kp = speed_kp;
    *ki = speed_ki;
}

/* ---- 齿槽补偿 ---- */

void foc_cog_enable(uint8_t en)
{
    cog_enabled = en;
    cog_ff = 0.0f;
}

uint8_t foc_cog_is_enabled(void)
{
    return cog_enabled;
}

void foc_cog_set_gain(float gain)
{
    cog_gain = gain;
}

float foc_cog_get_gain(void)
{
    return cog_gain;
}

void foc_cog_set_point(uint16_t idx, float val)
{
    if (idx < COG_LUT_SIZE) {
        cog_lut[idx] = val;
    }
}

float foc_cog_get_point(uint16_t idx)
{
    if (idx < COG_LUT_SIZE) {
        return cog_lut[idx];
    }
    return 0.0f;
}

uint16_t foc_cog_lut_size(void)
{
    return (uint16_t)COG_LUT_SIZE;
}

/* ---- 编码器角度校正 ---- */

void enc_cal_enable(uint8_t en)
{
    enc_cal_on = en;
}

uint8_t enc_cal_is_enabled(void)
{
    return enc_cal_on;
}

void enc_cal_reset(void)
{
    uint16_t i;
    for (i = 0; i < ENC_CAL_BINS; ++i) {
        enc_cal_sum[i] = 0;
        enc_cal_cnt[i] = 0;
    }
}

int32_t enc_cal_get_sum(uint16_t idx)
{
    return (idx < ENC_CAL_BINS) ? enc_cal_sum[idx] : 0;
}

uint32_t enc_cal_get_cnt(uint16_t idx)
{
    return (idx < ENC_CAL_BINS) ? enc_cal_cnt[idx] : 0;
}

void enc_corr_enable(uint8_t en)
{
    enc_corr_on = en;
}

uint8_t enc_corr_is_enabled(void)
{
    return enc_corr_on;
}

void enc_corr_set(uint16_t idx, float deg)
{
    if (idx < ENC_CAL_BINS) {
        float md = deg * 1000.0f;
        if (md >  32000.0f) md =  32000.0f;
        if (md < -32000.0f) md = -32000.0f;
        enc_corr_lut[idx] = (int16_t)md;
    }
}

float enc_corr_get(uint16_t idx)
{
    return (idx < ENC_CAL_BINS) ? ((float)enc_corr_lut[idx] * 0.001f) : 0.0f;
}

/* ---- PI 调参 ---- */

void foc_set_current_kp(float kp) { pi_d.kp = kp; pi_q.kp = kp; }
void foc_set_current_ki(float ki) { pi_d.ki = ki; pi_q.ki = ki; }

void foc_set_encoder_offset_deg(float deg)
{
    encoder_offset_rad = deg * DEG_TO_RAD;
}

float foc_get_encoder_offset_deg(void)
{
    return encoder_offset_rad * MOTOR_RAD_TO_DEG;
}

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
                   int *ia_out, int *ib_out, int *pos_out,
                   int *speed_ref_out, int *speed_est_out)
{
    __disable_irq();
    *iq_ref_out    = snap_iq_ref;
    *iq_out        = snap_iq;
    *id_out        = snap_id;
    *vd_out        = snap_vd;
    *vq_out        = snap_vq;
    *theta_out     = snap_theta;
    *ia_out        = snap_ia;
    *ib_out        = snap_ib;
    *pos_out       = snap_pos;
    *speed_ref_out = snap_speed_ref;
    *speed_est_out = snap_speed_est;
    trace_ready = 0;
    __enable_irq();
}

static void foc_trace_write_field(uint8_t* first, int value)
{
    if (*first == 0U) printf(",");
    printf("%d", value);
    *first = 0U;
}

/**
 * \brief           FOC trace 打印 (主循环调用)
 */
void foc_trace_poll(void)
{
    if (!trace_ready) return;

    int iqr, iq, id, vd, vq, th, ia, ib, pos, sr, se;
    foc_trace_get(&iqr, &iq, &id, &vd, &vq, &th, &ia, &ib, &pos, &sr, &se);
    {
        uint8_t first = 1U;

        printf("F:");
#if TRACE_SEND_IQ_REF
        foc_trace_write_field(&first, iqr);
#endif
#if TRACE_SEND_IQ
        foc_trace_write_field(&first, iq);
#endif
#if TRACE_SEND_ID
        foc_trace_write_field(&first, id);
#endif
#if TRACE_SEND_VD
        foc_trace_write_field(&first, vd);
#endif
#if TRACE_SEND_VQ
        foc_trace_write_field(&first, vq);
#endif
#if TRACE_SEND_THETA
        foc_trace_write_field(&first, th);
#endif
#if TRACE_SEND_IA
        foc_trace_write_field(&first, ia);
#endif
#if TRACE_SEND_IB
        foc_trace_write_field(&first, ib);
#endif
#if TRACE_SEND_POS
        foc_trace_write_field(&first, pos);
#endif
#if TRACE_SEND_SPEED_REF
        foc_trace_write_field(&first, sr);
#endif
#if TRACE_SEND_SPEED_EST
        foc_trace_write_field(&first, se);
#endif
        printf("\r\n");
    }
}
