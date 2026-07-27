/**
  ******************************************************************************
  * @file           : app_foc.c
  * @brief          : 闭环 FOC 矢量控制核心
  * @note           : 电流环 20kHz: Clarke→Park→电流PI→电压圆限制→逆Park→SVPWM→HRTIM
  *                   速度环 1kHz (20kHz/FOC_SPEED_LOOP_DIV 降频), 含斜坡软启动.
  *                   与开环模块互斥: foc_is_enabled() 时由 foc_step() 接管 PWM.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
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

/* -------------------------------------------------------------------------- */
/*  数学常量 (避免重复计算)                                                     */
/* -------------------------------------------------------------------------- */

#define PI              3.14159265358979323846f
#define TWO_PI          6.28318530717958647692f
#define INV_SQRT3       0.57735026918962576451f   /* 1/sqrt(3)                */
#define SQRT3_2         0.86602540378443864676f   /* sqrt(3)/2                */
#define DEG_TO_RAD      0.01745329251994329577f   /* PI/180                   */
#define RAD_TO_DEG      57.29577951308232087680f  /* 180/PI                   */

/* -------------------------------------------------------------------------- */
/*  PI 控制器                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * \brief           初始化 PI 控制器
 */
static void pi_init(foc_pi_t *pi, float Kp, float Ki, float Ts,
                    float out_min, float out_max)
{
    pi->Kp       = Kp;
    pi->Ki       = Ki;
    pi->Ts       = Ts;
    pi->integral = 0.0f;
    pi->output   = 0.0f;
    pi->out_min  = out_min;
    pi->out_max  = out_max;
}

/**
 * \brief           重置 PI 积分和输出
 */
static void pi_reset(foc_pi_t *pi)
{
    pi->integral = 0.0f;
    pi->output   = 0.0f;
}

/**
 * \brief           PI 控制器更新 (带 anti-windup)
 * \param[in,out]   pi     PI 状态
 * \param[in]       error  误差 = 参考 - 反馈
 * \return          控制器输出 (已钳位)
 *
 * \note            线性区: integral += Ki * error * Ts (正向欧拉)
 *                  饱和区: integral = u_clamped - Kp*error (反算保持)
 */
static float pi_update(foc_pi_t *pi, float error)
{
    float P_term = pi->Kp * error;
    float u_tent = P_term + pi->integral;

    /* 输出钳位 */
    float u_out;
    if (u_tent > pi->out_max) {
        u_out = pi->out_max;
    } else if (u_tent < pi->out_min) {
        u_out = pi->out_min;
    } else {
        u_out = u_tent;
    }

    /* Anti-windup 积分更新 */
    if (u_out != u_tent) {
        /* 饱和: 反算积分使输出恰好等于钳位值 */
        pi->integral = u_out - P_term;
    } else {
        /* 线性: 正向欧拉累积 */
        pi->integral += pi->Ki * error * pi->Ts;
    }

    /* 安全钳位 (防止数值异常) */
    if (pi->integral > pi->out_max) pi->integral = pi->out_max;
    if (pi->integral < pi->out_min) pi->integral = pi->out_min;

    pi->output = u_out;
    return u_out;
}

/* -------------------------------------------------------------------------- */
/*  静态状态变量                                                                */
/* -------------------------------------------------------------------------- */

/** PI 控制器 */
static foc_pi_t  pi_d;          /**< Id 电流环 PI                        */
static foc_pi_t  pi_q;          /**< Iq 电流环 PI                        */
static foc_pi_t  pi_speed;      /**< 速度环 PI                           */

/** 控制模式与状态 */
static foc_mode_t control_mode = FOC_MODE_TORQUE;
static foc_state_t foc_state   = FOC_STATE_IDLE;

/** 参考值 */
static float i_d_ref   = 0.0f;  /**< Id 目标 (A), SPMSM 固定为 0        */
static float i_q_ref   = 0.0f;  /**< Iq 目标 (A), 扭矩模式直接设         */
static float speed_ref      = 0.0f;  /**< 转速斜坡实际值 (RPM), 速度PI用   */
static float speed_ref_cmd  = 0.0f;  /**< 转速斜坡目标值 (RPM), 用户设定   */

/** 速度环降频计数器 */
static uint8_t speed_loop_cnt = 0;   /**< 速度环分频计数: 0..FOC_SPEED_LOOP_DIV-1 */

/** 编码器故障保护 */
static uint8_t enc_err_count  = 0;   /**< 连续编码器失败计数                  */

/** 诊断输出 */
static uint8_t trace_enabled  = 1;   /**< 1=开启 10Hz FOC 诊断输出, 默认开    */
static uint16_t diag_cnt      = 0;   /**< 诊断输出降频计数                    */
static uint8_t last_angle_ok  = 0;   /**< 最近一次 foc_update_angle 结果      */

/** 速度环旁路测试: 非零时跳过速度 PI, 直接用此值作为 i_q_ref */
static float speed_bypass_iq  = 0.0f;

/** 变换中间变量 */
static float i_alpha, i_beta;   /**< Clarke 输出 (A)                     */
static float i_d, i_q;          /**< Park 输出 (A)                       */
static float v_d, v_q;          /**< PI 输出电压 (归一化 -1~1)           */
static float v_alpha, v_beta;   /**< 逆 Park 输出                        */
static float volt_a, volt_b, volt_c; /**< 最近三相电压 (调试用)          */

/** 编码器与角度 */
static float theta_elec      = 0.0f;   /**< 当前电角度 (rad, 含offset)   */
static float encoder_offset  = 0.0f;   /**< 编码器安装偏移 (rad, 电角度)  */

/** 速度估算 */
static float speed_mech_rpm  = 0.0f;   /**< 滤波后机械转速 (RPM)          */
static float speed_mech_rad  = 0.0f;   /**< 滤波后机械转速 (rad/s)        */

/** 电流原始值 (mA, 调试用) */
static float ia_ma, ib_ma, ic_ma;

/** 齿槽补偿状态 (方案 C: 谐波 + 低速 boost) */
static float  cog_amps    = 0.0f;   /**< 谐波幅值 (A), 0=禁用        */
static float  cog_n       = 6.0f;   /**< 每电周期齿槽波数            */
static float  cog_phase   = 0.0f;   /**< 谐波相位 (rad)             */
static float  cog_boost   = 0.0f;   /**< 低速 boost 电流 (A)        */
static float  cog_boost_th = 200.0f;/**< 低速 boost 阈值 (RPM)      */

/* -------------------------------------------------------------------------- */
/*  私有辅助函数                                                               */
/* -------------------------------------------------------------------------- */

/**
 * \brief           编码器角度 → 电角度 (含安装偏移 + 噪声过滤)
 * \return          成功返回 1, 失败时用速度预测角度并返回 0
 * \note            不使用 CRC (MT6701 数据手册标明 CRC 不可靠).
 *                  仅靠 delta 物理限幅过滤 SPI 噪声.
 *                  失败时用上次瞬时速度预测角度, 避免冻结导致 Park 变换跳变.
 */
static uint8_t foc_update_angle(void)
{
    static float   prev_valid_theta = 0.0f;
    static uint8_t had_valid        = 0;

    mt6701_data_t enc = mt6701_read_angle();
    uint8_t ok = 0;
    float new_theta = 0.0f;

    /* 机械角度 → 电角度 (极对数=7, 电角度可达 2520°, 必须 fmodf) */
    float enc_elec_deg = fmodf(enc.angle_deg * (float)FOC_POLE_PAIRS, 360.0f);
    if (enc_elec_deg < 0.0f) enc_elec_deg += 360.0f;

    float raw_theta = fmodf(enc_elec_deg * DEG_TO_RAD + encoder_offset, TWO_PI);
    if (raw_theta < 0.0f) raw_theta += TWO_PI;

    /* 最简角度滤波: |delta| < 1.0 rad → 接受(返回1), 否则预测(返回0).
     * 永不停机, 不重置 had_valid. */
    if (had_valid) {
        float delta = raw_theta - prev_valid_theta;
        if (delta >  PI) delta -= TWO_PI;
        if (delta < -PI) delta += TWO_PI;

        if (delta > FOC_ANGLE_DELTA_MAX || delta < -FOC_ANGLE_DELTA_MAX) {
            /* 跳变: 预测角度, 返回 0 跳过速度估算 */
            theta_elec += (delta > 0.0f ? 1.0f : -1.0f);
            if (theta_elec >= TWO_PI) theta_elec -= TWO_PI;
            if (theta_elec < 0.0f)   theta_elec += TWO_PI;
            return 0;
        }
    }

    /* 接受 */
    theta_elec       = raw_theta;
    prev_valid_theta = raw_theta;
    had_valid        = 1;
    return 1;
}

/**
 * \brief           从电角度差分估算机械转速 (1kHz 调用)
 * \note            调用者保证 delta 是 1ms 周期角度增量.
 *                  通过 EMA 低通滤波抑制微分噪声.
 *                  速率限制 ±500 RPM/step (1ms) 防止角度跳变.
 */
static void foc_update_speed_from_delta(float delta_1ms)
{
    /* NaN/Inf 检测: delta 异常时跳过本次速度更新 */
    if (!isfinite(delta_1ms)) {
        return;
    }

    /* 处理 0↔2π 回绕 */
    if (delta_1ms > PI)       delta_1ms -= TWO_PI;
    if (delta_1ms < -PI)      delta_1ms += TWO_PI;

    /* 1kHz → 电角速度 → 机械角速度 → RPM */
    float speed_elec_rad_s = delta_1ms * FOC_SPEED_FREQ;
    float speed_mech_tmp   = speed_elec_rad_s / (float)FOC_POLE_PAIRS;
    float rpm_tmp          = speed_mech_tmp * (60.0f / TWO_PI);

    /* 速率限制: ±500 RPM/step */
    float rpm_old   = speed_mech_rad * (60.0f / TWO_PI);
    if (rpm_tmp - rpm_old >  FOC_SPEED_RATE_LIMIT) rpm_tmp = rpm_old + FOC_SPEED_RATE_LIMIT;
    if (rpm_tmp - rpm_old < -FOC_SPEED_RATE_LIMIT) rpm_tmp = rpm_old - FOC_SPEED_RATE_LIMIT;
    speed_mech_tmp = rpm_tmp / (60.0f / TWO_PI);

    /* EMA 低通滤波 */
    speed_mech_rad += FOC_SPEED_LPF_ALPHA * (speed_mech_tmp - speed_mech_rad);

    /* 转换为 RPM */
    speed_mech_rpm = speed_mech_rad * (60.0f / TWO_PI);
}

/**
 * \brief           电压圆限制: 确保 Vd,Vq 矢量不超出调制范围
 * \note            超出时等比缩放 Vd,Vq, 并对两个电流 PI 做积分反算.
 *                  防止过调制导致的电流失控.
 */
static void foc_voltage_limit(void)
{
    float v_mag_sq = v_d * v_d + v_q * v_q;
    float v_max_sq = FOC_V_MAX_NORM * FOC_V_MAX_NORM;

    if (v_mag_sq > v_max_sq) {
        float scale = FOC_V_MAX_NORM / sqrtf(v_mag_sq);
        v_d *= scale;
        v_q *= scale;

        /* 积分反算: 使 PI 积分与缩放后的输出一致 */
        float err_d = i_d_ref - i_d;
        float err_q = i_q_ref - i_q;
        pi_d.integral = v_d - pi_d.Kp * err_d;
        pi_q.integral = v_q - pi_q.Kp * err_q;

        /* 安全钳位 */
        if (pi_d.integral > pi_d.out_max) pi_d.integral = pi_d.out_max;
        if (pi_d.integral < pi_d.out_min) pi_d.integral = pi_d.out_min;
        if (pi_q.integral > pi_q.out_max) pi_q.integral = pi_q.out_max;
        if (pi_q.integral < pi_q.out_min) pi_q.integral = pi_q.out_min;
    }
}

/**
 * \brief           SVPWM + HRTIM 寄存器写入
 * \note            逆 Clarke → 三次谐波注入 → CMP1 映射
 *
 * 物理接线对应关系:
 *   Timer B (PA10/PA11) → 电机 A 相
 *   Timer A (PA8/PA9)   → 电机 B 相
 *   Timer D (PB14/PB15) → 电机 C 相
 */
static void foc_svpwm_write(void)
{
    /* 逆 Clarke: Vαβ → Va, Vb, Vc (与正 Clarke i_alpha=ia, i_beta=(ia+2ib)/√3 配对) */
    volt_a =  v_alpha;
    volt_b = -0.5f * v_alpha + SQRT3_2 * v_beta;
    volt_c = -0.5f * v_alpha - SQRT3_2 * v_beta;

    /* SVPWM 注入 + CMP 映射 + HRTIM 写入 (共享函数, 始终启用 SVPWM) */
    svpwm_write(volt_a, volt_b, volt_c, 1);
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * \brief           初始化 FOC 模块
 * \note            必须在 openloop_init() 之后调用 (读取 encoder_offset).
 *                  初始化为空闲状态, 无 PWM 输出.
 */
void foc_init(void)
{
    /* 从开环模块获取编码器安装偏移 (rad, 电角度) */
    encoder_offset = openloop_get_offset();
    printf("FOC: encoder offset = %d deg (from openloop)\r\n",
           (int)(encoder_offset * RAD_TO_DEG));

    /* 初始化电流 PI (Id 和 Iq 使用相同增益) */
    pi_init(&pi_d, FOC_DEFAULT_KP_CURRENT, FOC_DEFAULT_KI_CURRENT, FOC_TS,
            FOC_PI_CURRENT_OUT_MIN, FOC_PI_CURRENT_OUT_MAX);
    pi_init(&pi_q, FOC_DEFAULT_KP_CURRENT, FOC_DEFAULT_KI_CURRENT, FOC_TS,
            FOC_PI_CURRENT_OUT_MIN, FOC_PI_CURRENT_OUT_MAX);

    /* 初始化速度 PI (Ts = 1kHz降频后的采样周期) */
    pi_init(&pi_speed, FOC_DEFAULT_KP_SPEED, FOC_DEFAULT_KI_SPEED, FOC_SPEED_TS,
            -FOC_IQ_MAX, FOC_IQ_MAX);

    /* 状态清零 */
    i_d_ref       = 0.0f;
    i_q_ref       = 0.0f;
    speed_ref      = 0.0f;
    speed_ref_cmd  = 0.0f;
    speed_loop_cnt = 0;
    enc_err_count  = 0;
    trace_enabled  = 1;  /* 默认开启诊断输出 */
    diag_cnt       = 0;
    last_angle_ok  = 0;
    theta_elec    = 0.0f;
    speed_mech_rpm = 0.0f;
    speed_mech_rad = 0.0f;
    v_d = v_q     = 0.0f;
    volt_a = volt_b = volt_c = 0.0f;
    control_mode  = FOC_MODE_TORQUE;
    foc_state     = FOC_STATE_IDLE;

    /* 恢复默认滤波 (死区 ~1.5μs 仍需适度滤波) */
    curr_sense_set_filter_alpha(0.15f);

    printf("FOC: init done, mode=torque, state=idle\r\n");
    printf("FOC: current PI Kp=%d Ki=%d\r\n",
           (int)(FOC_DEFAULT_KP_CURRENT * 1000.0f),
           (int)(FOC_DEFAULT_KI_CURRENT * 1000.0f));
}

/**
 * \brief           使能 FOC 闭环控制
 * \note            从零电压开始, Id=Iq 参考 = 0.
 */
void foc_start(void)
{
    if (foc_state == FOC_STATE_RUNNING) return;

    /* 重新读取编码器偏移 (对齐校准后可能已更新) */
    encoder_offset = openloop_get_offset();
    printf("FOC: encoder offset = %d deg\r\n",
           (int)(encoder_offset * RAD_TO_DEG));

    /* 重置 PI 积分 */
    pi_reset(&pi_d);
    pi_reset(&pi_q);
    pi_reset(&pi_speed);

    /* 同步编码器当前角度 (重试直到有效, 防止启动时 Park 角错位) */
    {
        int retry = 0;
        while (!foc_update_angle() && retry < FOC_ENC_INIT_RETRY) { retry++; }
        if (retry >= FOC_ENC_INIT_RETRY) {
            printf("FOC: encoder init failed, start aborted!\r\n");
            return;
        }
    }
    /* 零参考 */
    i_q_ref   = 0.0f;
    i_d_ref   = 0.0f;
    speed_ref     = 0.0f;
    speed_ref_cmd = 0.0f;
    speed_loop_cnt = 0;
    enc_err_count  = 0;
    diag_cnt       = 0;
    last_angle_ok  = 1;
    v_d = v_q = 0.0f;

    foc_state = FOC_STATE_RUNNING;
    printf("FOC: started (mode=%s)\r\n",
           control_mode == FOC_MODE_TORQUE ? "torque" : "speed");
}

/**
 * \brief           禁用 FOC, 立即输出电压到零
 */
void foc_stop(void)
{
    foc_state = FOC_STATE_IDLE;

    /* 电压零 */
    v_d = 0.0f;
    v_q = 0.0f;
    v_alpha = 0.0f;
    v_beta  = 0.0f;
    i_q_ref = 0.0f;
    speed_ref_cmd = 0.0f;
    speed_loop_cnt = 0;
    enc_err_count  = 0;

    /* 重置 PI */
    pi_reset(&pi_d);
    pi_reset(&pi_q);
    pi_reset(&pi_speed);

    /* 写零电压到 PWM (Vα=Vβ=0 → CMP=center) */
    volt_a = 0.0f;
    volt_b = 0.0f;
    volt_c = 0.0f;
    svpwm_write(0.0f, 0.0f, 0.0f, 1);

    printf("FOC: stopped\r\n");
}

/* --- 控制模式 --- */

void foc_set_torque_mode(void)
{
    control_mode = FOC_MODE_TORQUE;
    i_q_ref = 0.0f;  /* 切换时清零 Iq 参考 */
    pi_reset(&pi_speed);
    printf("FOC: torque mode\r\n");
}

void foc_set_speed_mode(void)
{
    control_mode = FOC_MODE_SPEED;
    i_q_ref = 0.0f;
    speed_ref     = 0.0f;
    speed_ref_cmd = 0.0f;
    speed_loop_cnt = 0;
    enc_err_count  = 0;
    pi_reset(&pi_speed);
    printf("FOC: speed mode\r\n");
}

foc_mode_t foc_get_mode(void)
{
    return control_mode;
}

/* --- 参考值 --- */

void foc_set_iq_ref(float iq_amps)
{
    if (iq_amps > FOC_IQ_MAX)  iq_amps = FOC_IQ_MAX;
    if (iq_amps < -FOC_IQ_MAX) iq_amps = -FOC_IQ_MAX;
    i_q_ref = iq_amps;
}

void foc_set_speed_ref(float rpm)
{
    /* 限幅: ±10000 RPM 合理范围 */
    if (rpm >  FOC_SPEED_REF_MAX) rpm =  FOC_SPEED_REF_MAX;
    if (rpm < -FOC_SPEED_REF_MAX) rpm = -FOC_SPEED_REF_MAX;
    speed_ref_cmd = rpm;  /* 斜坡目标, speed_ref 由 foc_step() 逐步逼近 */
}

float foc_get_speed_ref_cmd(void)
{
    return speed_ref_cmd;
}

/* --- 编码器 Offset 在线调整 --- */

void foc_set_encoder_offset_deg(float offset_deg)
{
    encoder_offset = offset_deg * DEG_TO_RAD;
    printf("FOC: encoder offset = %d deg\r\n", (int)offset_deg);
}

float foc_get_encoder_offset_deg(void)
{
    return encoder_offset * RAD_TO_DEG;
}

/* --- PI 整定 --- */

void foc_set_current_pi(float Kp, float Ki)
{
    pi_d.Kp = Kp;
    pi_d.Ki = Ki;
    pi_q.Kp = Kp;
    pi_q.Ki = Ki;
    printf("FOC: current PI Kp=%d Ki=%d\r\n", (int)(Kp * 1000.0f), (int)(Ki * 1000.0f));
}

void foc_get_current_pi(float *Kp, float *Ki)
{
    *Kp = pi_d.Kp;
    *Ki = pi_d.Ki;
}

void foc_set_speed_pi(float Kp, float Ki)
{
    pi_speed.Kp = Kp;
    pi_speed.Ki = Ki;
    pi_speed.Ts = FOC_SPEED_TS;  /* 速度环 Ts 固定 1kHz, 不与电流环混淆 */
    printf("FOC: speed PI Kp=%d Ki=%d\r\n", (int)(Kp * 1000.0f), (int)(Ki * 1000.0f));
}

void foc_get_speed_pi(float *Kp, float *Ki)
{
    *Kp = pi_speed.Kp;
    *Ki = pi_speed.Ki;
}

/* --- 齿槽补偿 --- */

void foc_cogging_set_harmonic(float amps, float n_cycles, float phase_deg)
{
    cog_amps  = amps;
    cog_n     = (n_cycles > 0.0f) ? n_cycles : 6.0f;
    cog_phase = phase_deg * DEG_TO_RAD;
    printf("FOC: cog harmonic A=%d mA n=%d ph=%d deg\r\n",
           (int)(cog_amps * 1000.0f), (int)cog_n, (int)phase_deg);
}

void foc_cogging_get_harmonic(float *amps, float *n_cycles, float *phase_deg)
{
    *amps      = cog_amps;
    *n_cycles  = cog_n;
    *phase_deg = cog_phase * RAD_TO_DEG;
}

void foc_cogging_set_boost(float boost_amps, float threshold_rpm)
{
    cog_boost   = (boost_amps >= 0.0f) ? boost_amps : 0.0f;
    cog_boost_th = (threshold_rpm > 0.0f) ? threshold_rpm : 200.0f;
    printf("FOC: cog boost=%d mA th=%d RPM\r\n",
           (int)(cog_boost * 1000.0f), (int)cog_boost_th);
}

void foc_cogging_get_boost(float *boost_amps, float *threshold_rpm)
{
    *boost_amps     = cog_boost;
    *threshold_rpm  = cog_boost_th;
}

/* --- 状态查询 --- */

uint8_t foc_is_enabled(void)
{
    return (foc_state == FOC_STATE_RUNNING) ? 1 : 0;
}

void foc_get_state(float *id, float *iq, float *vq, float *speed_rpm)
{
    *id        = i_d;
    *iq        = i_q;
    *vq        = v_q;
    *speed_rpm = speed_mech_rpm;
}

void foc_get_voltages(float *va, float *vb, float *vc)
{
    *va = volt_a;
    *vb = volt_b;
    *vc = volt_c;
}

foc_state_t foc_get_run_state(void)
{
    return foc_state;
}

uint8_t foc_get_enc_err_count(void)
{
    return enc_err_count;
}

void foc_set_trace(uint8_t on)
{
    trace_enabled = (on != 0) ? 1 : 0;
    diag_cnt      = 0;
    printf("FOC: trace %s\r\n", trace_enabled ? "ON" : "OFF");
}

void foc_speed_bypass(float iq)
{
    speed_bypass_iq = iq;
    printf("FOC: speed bypass Iq=%d mA\r\n", (int)(iq * 1000.0f));
}

/* -------------------------------------------------------------------------- */
/*  快速控制环 (20kHz)                                                          */
/* -------------------------------------------------------------------------- */

/**
 * \brief           每 PWM 周期调用: 完整 FOC 算法链
 * \note            在 curr_sense_data_ready() 后调用, 保证 20kHz 同步.
 *
 * 算法链:
 *   1. 读取三相电流 (mA→A)
 *   2. 读取编码器电角度
 *   3. 编码器故障计数 → 连续超限则紧急停机
 *   4. 速度环降频计数 → 每 FOC_SPEED_LOOP_DIV 次:
 *      a. 1kHz 速度估算 (1ms 角度差分)
 *      b. 速度斜坡 (speed_ref → speed_ref_cmd)
 *      c. 速度 PI (仅速度模式)
 *      d. 诊断输出 (若 trace 开启)
 *   5. Clarke 变换 (Ia,Ib → Iα,Iβ)
 *   6. Park 变换 (Iα,Iβ → Id,Iq)
 *   7. 电流 PI (Id→Vd, Iq→Vq)
 *   8. 电压圆限制 (防过调制 + 积分反算)
 *   9. 逆 Park 变换 (Vd,Vq → Vα,Vβ)
 *  10. SVPWM + HRTIM CMP1 写入
 */
void foc_step(void)
{
    if (foc_state != FOC_STATE_RUNNING) return;

    /* ---- 1. 读取三相电流 (mA → A) ---- */
    curr_sense_get_all(&ia_ma, &ib_ma, &ic_ma);
    float ia = ia_ma * 0.001f;
    float ib = ib_ma * 0.001f;

    /* ---- 2. 读取编码器电角度 ---- */
    uint8_t angle_ok = foc_update_angle();
    last_angle_ok = angle_ok;

    /* ---- 3. 编码器故障保护 ----
     * 连续失败 > FOC_ENC_ERR_MAX (1ms) → 紧急停机.
     * 不允许用预测角度继续闭环, 会导致 Park 角错误 → Iq 方向错误 → 失磁. */
    if (!angle_ok) {
        enc_err_count++;
        if (enc_err_count > FOC_ENC_ERR_MAX) {
            printf("FOC: ENCODER LOST! (%d consecutive fails)\r\n", enc_err_count);
            foc_stop();
            return;
        }
    } else {
        enc_err_count = 0;
    }

    /* ---- 4. 速度环 (1kHz 降频运行) ---- */
    speed_loop_cnt++;
    if (speed_loop_cnt >= FOC_SPEED_LOOP_DIV) {
        speed_loop_cnt = 0;

        /* ---- 4a. 速度估算 (1kHz, 1ms 角度差分) ----
         * 使用独立的 theta_for_speed 变量, 仅在 1kHz tick 采样.
         * 避免 20kHz 瞬时差分在低速时的量化噪声. */
        {
            static float  theta_for_speed_prev = 0.0f;
            static uint8_t speed_theta_inited  = 0;

            if (!speed_theta_inited) {
                theta_for_speed_prev = theta_elec;
                speed_theta_inited   = 1;
            }

            float delta_1ms = theta_elec - theta_for_speed_prev;
            theta_for_speed_prev = theta_elec;

            /* 仅在最近有有效编码器读数时才更新速度 */
            if (enc_err_count == 0) {
                foc_update_speed_from_delta(delta_1ms);
            }
        }

        /* ---- 4b. 速度斜坡: speed_ref 逐步逼近 speed_ref_cmd ---- */
        if (speed_ref < speed_ref_cmd) {
            speed_ref += FOC_SPEED_RAMP;
            if (speed_ref > speed_ref_cmd) speed_ref = speed_ref_cmd;
        } else if (speed_ref > speed_ref_cmd) {
            speed_ref -= FOC_SPEED_RAMP;
            if (speed_ref < speed_ref_cmd) speed_ref = speed_ref_cmd;
        }

        /* ---- 4c. 速度 PI (仅速度模式) ---- */
        if (control_mode == FOC_MODE_SPEED) {
            /* 旁路测试: speed_bypass_iq != 0 时跳过速度PI, 直接用固定 Iq */
            if (speed_bypass_iq != 0.0f) {
                i_q_ref = speed_bypass_iq;
            } else {
                float speed_err = speed_ref - speed_mech_rpm;

                /* NaN/Inf 检测: 一旦捕获立即输出告警 */
                if (!isfinite(speed_mech_rpm) || !isfinite(speed_ref)) {
                    printf("SPD_NAN: rpm=%d ref=%d\r\n",
                           (int)speed_mech_rpm, (int)speed_ref);
                }
                if (!isfinite(pi_speed.integral) || !isfinite(pi_speed.output)) {
                    printf("PI_NAN: int=%d out=%d\r\n",
                           (int)(pi_speed.integral * 1000.0f),
                           (int)(pi_speed.output * 1000.0f));
                }

                i_q_ref = pi_update(&pi_speed, speed_err);
            }

            /* 钳位 Iq 到额定范围 */
            if (i_q_ref > FOC_IQ_MAX)  i_q_ref = FOC_IQ_MAX;
            if (i_q_ref < -FOC_IQ_MAX) i_q_ref = -FOC_IQ_MAX;
        }

        /* ---- 4d. 诊断输出 (10Hz, 每 100 个速度 tick = 100ms) ---- */
        if (trace_enabled) {
            diag_cnt++;
            if (diag_cnt >= FOC_DIAG_INTERVAL) {
                diag_cnt = 0;
                printf("D:%d,%d,%d,%d,%d,%d,%d,%d ia=%d ib=%d\r\n",
                       (int)speed_mech_rpm,
                       (int)(i_q_ref * 1000.0f),
                       (int)(i_q * 1000.0f),
                       (int)(i_d * 1000.0f),
                       (int)(v_q * 1000.0f),
                       (int)(theta_elec * RAD_TO_DEG),
                       last_angle_ok,
                       enc_err_count,
                       (int)(ia_ma),
                       (int)(ib_ma));
            }
        }
    }

    /* ---- 5. 齿槽补偿 (谐波 + 低速 boost) ---- */
    float iq_ref_eff = i_q_ref;
    {
        if (cog_amps != 0.0f) {
            iq_ref_eff += cog_amps * sinf(cog_n * theta_elec + cog_phase);
        }
        if (cog_boost > 0.0f) {
            float abs_spd = (speed_mech_rpm >= 0.0f) ? speed_mech_rpm : -speed_mech_rpm;
            if (abs_spd < cog_boost_th) {
                iq_ref_eff += cog_boost * (1.0f - abs_spd / cog_boost_th);
            }
        }
        if (iq_ref_eff > FOC_IQ_MAX)  iq_ref_eff = FOC_IQ_MAX;
        if (iq_ref_eff < -FOC_IQ_MAX) iq_ref_eff = -FOC_IQ_MAX;
    }

    /* ---- 6. Clarke 变换 (幅值不变) ---- */
    i_alpha = ia;
    i_beta  = (ia + 2.0f * ib) * INV_SQRT3;

    /* ---- 7. Park 变换 ---- */
    float sin_th = sinf(theta_elec);
    float cos_th = cosf(theta_elec);
    i_d =  i_alpha * cos_th + i_beta * sin_th;
    i_q = -i_alpha * sin_th + i_beta * cos_th;

    /* ---- 8. 电流 PI ---- */
    float err_d = i_d_ref - i_d;
    float err_q = iq_ref_eff - i_q;
    v_d = pi_update(&pi_d, err_d);
    v_q = pi_update(&pi_q, err_q);

    /* ---- 9. 电压圆限制 ---- */
    foc_voltage_limit();

    /* ---- 10. 逆 Park 变换 ---- */
    v_alpha = v_d * cos_th - v_q * sin_th;
    v_beta  = v_d * sin_th + v_q * cos_th;

    /* ---- 11. SVPWM + HRTIM CMP1 写入 ---- */
    foc_svpwm_write();
}
