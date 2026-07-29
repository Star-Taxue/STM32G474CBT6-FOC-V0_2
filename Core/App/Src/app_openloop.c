/**
  ******************************************************************************
  * @file           : app_openloop.c
  * @brief          : 开环电压驱动 (V/F 标量控制)
  * @note           : 生成三相对称正弦电压, 通过 HRTIM CMP1 更新 PWM 占空比.
  *                   包含软启动 ramp 和正弦查表.
  *                   每 PWM 周期在 ADC 回调中调用 openloop_step().
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hrtim.h"
#include "app_openloop.h"
#include "motor_config.h"
#include "svpwm.h"
#define TWO_PI        MOTOR_TWO_PI
#define POLE_PAIRS    MOTOR_POLE_PAIRS
#define PWM_FREQ_HZ   MOTOR_PWM_FREQ

#include "mt6701.h"
#include "app_current_sense.h"
#include <math.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/

/*
 * 编码器安装偏差 (度, 电角度).
 * 校准命令: align 0.3 → 等1s → offset → 把打印值填到这里.
 */
#ifndef ENCODER_OFFSET_DEG
#define ENCODER_OFFSET_DEG  16   /* 106-90, 试 dq 轴对齐 */
#endif

/** 默认参数 */
#define SOFT_START_RATE  0.0003f    /* K ramp 速率 / PWM 周期               */
#define DEFAULT_FREQ_HZ  10.0f      /* 默认电频率 (Hz)                      */
#define DEFAULT_VOLTAGE   0.10f     /* 默认调制系数                          */

/** 数学常量 (模块局部) */
#define PI_OVER_2        1.570796327f   /* π/2, A 相对齐角             */
#define PHASE_120        2.094395102f   /* 2*PI/3 */
#define PHASE_240        4.188790205f   /* 4*PI/3 */
#define SIN_TABLE_SIZE   256

/* Private variables ---------------------------------------------------------*/

/** 正弦查找表 (256 点, 覆盖 0 ~ 2π) */
static float sin_table[SIN_TABLE_SIZE];

/** 运行时状态 */
static float theta;          /* 当前电角度 (rad), 0 ~ 2π             */
static float target_K;       /* 目标调制系数 (用户设定)               */
static float current_K;      /* 当前调制系数 (ramp 过渡值)            */
static float theta_step;     /* 角度增量 / PWM 周期 (由频率决定)     */
static float target_freq;    /* 目标电频率 (Hz)                      */
static uint8_t table_ready;  /* 正弦表就绪标志                        */
static uint8_t lock_mode;    /* 对准锁定模式 (1=锁定θ, 不推进角度)    */
static uint8_t svpwm_enable; /* SVPWM 模式 (1=马鞍波, 0=纯正弦)      */
static float volt_a, volt_b, volt_c; /* 最近一次输出电压 (调试用)     */
static float encoder_offset; /* 编码器安装偏移 (rad, 电角度)           */
static uint16_t trace_cnt;       /* trace 降频计数 (与 FOC 共享 TRACE_DIV) */

/* Trace 快照 (控制环捕获, 主循环打印, 不在 20kHz 路径调 printf) */
static volatile uint8_t ol_trace_ready;
static int      ol_snap_theta;
static int      ol_snap_K;
static int      ol_snap_va, ol_snap_vb, ol_snap_vc;
static uint16_t ol_snap_adc[3];
static int      ol_snap_enc;

/* Private function prototypes -----------------------------------------------*/

/**
 * \brief           构建正弦查找表
 * \note            启动时调用一次, 使用标准库 sinf 计算 256 个点.
 */
static void build_sin_table(void)
{
    uint16_t i;
    for (i = 0; i < SIN_TABLE_SIZE; i++) {
        float angle = TWO_PI * (float)i / (float)SIN_TABLE_SIZE;
        sin_table[i] = sinf(angle);
    }
    table_ready = 1;
}

/**
 * \brief           快速正弦查表
 * \param[in]       angle  角度 (rad), 自动归一到 [0, 2π)
 * \return          sin(angle)
 */
static inline float fast_sin(float angle)
{
    /* 归一到 [0, 2π) */
    while (angle < 0.0f)       angle += TWO_PI;
    while (angle >= TWO_PI)    angle -= TWO_PI;

    /* 线性插值查表 */
    float idx_f = angle * ((float)SIN_TABLE_SIZE / TWO_PI);
    uint16_t idx = (uint16_t)idx_f;
    float frac = idx_f - (float)idx;

    uint16_t next = (idx + 1) & (SIN_TABLE_SIZE - 1);
    return sin_table[idx] + frac * (sin_table[next] - sin_table[idx]);
}

/* Public API ----------------------------------------------------------------*/

/**
 * \brief           初始化开环驱动
 */
void openloop_init(void)
{
    build_sin_table();

    theta          = 0.0f;
    target_K       = 0.0f;
    current_K      = 0.0f;
    target_freq    = DEFAULT_FREQ_HZ;
    theta_step     = TWO_PI * target_freq / PWM_FREQ_HZ;
    lock_mode      = 0;
    svpwm_enable   = 1;   /* 默认 SVPWM */
    encoder_offset = (float)ENCODER_OFFSET_DEG * TWO_PI / 360.0f;
    trace_cnt      = 0;
    printf("encoder offset loaded: %d deg\r\n", (int)ENCODER_OFFSET_DEG);
}

/**
 * \brief           设置目标电压
 */
void openloop_set_voltage(float k)
{
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    target_K = k;
}

/**
 * \brief           设置目标电频率
 */
void openloop_set_freq(float freq_hz)
{
    if (freq_hz < 0.1f)  freq_hz = 0.1f;
    if (freq_hz > 500.0f) freq_hz = 500.0f;
    target_freq = freq_hz;
    theta_step  = TWO_PI * target_freq / PWM_FREQ_HZ;
}

/**
 * \brief           每 PWM 周期调用: 更新角度、占空比
 * \note            在 curr_sense_data_ready() 回调中调用,
 *                  保证与 ADC 采样同步 (20kHz).
 */
void openloop_step(void)
{
    if (!table_ready) return;

    /* --- 软启动: ramp current_K 趋近 target_K --- */
    if (current_K < target_K) {
        current_K += SOFT_START_RATE;
        if (current_K > target_K) current_K = target_K;
    } else if (current_K > target_K) {
        current_K -= SOFT_START_RATE;
        if (current_K < target_K) current_K = target_K;
    }

    /* 对准锁定模式: θ 固定为 90° (A 相), 不推进 */
    if (lock_mode) {
        theta = PI_OVER_2;
    }

    /* K ≈ 0 时输出零电压 (CMP = CENTER, 50% duty), 防止上次电压残留 */
    if (current_K < 0.0005f) {
        svpwm_write(0.0f, 0.0f, 0.0f, 0);
        if (!lock_mode) {
            theta += theta_step;
            if (theta >= TWO_PI) theta -= TWO_PI;
        }
        return;
    }

    /* --- 三相对称正弦电压 --- */
    float va = current_K * fast_sin(theta);
    float vb = current_K * fast_sin(theta + PHASE_120);
    float vc = current_K * fast_sin(theta + PHASE_240);

    /* 保存电压供调试输出 */
    volt_a = va;
    volt_b = vb;
    volt_c = vc;

    /* SVPWM + CMP 映射 + HRTIM 写入 (共享函数) */
    svpwm_write(va, vb, vc, svpwm_enable);

    /* --- 统一 Trace 快照 (与 FOC 共享 TRACE_DIV=200Hz, 不调 printf) --- */
    extern uint8_t g_trace_enabled;
    if (g_trace_enabled) {
        trace_cnt++;
        if (trace_cnt >= TRACE_DIV) {
            trace_cnt = 0;
            uint16_t adc_raw[3];
            curr_sense_get_raw_adc(&adc_raw[0], &adc_raw[1], &adc_raw[2]);
            float enc_elec = mt6701_read_angle().angle_deg * (float)POLE_PAIRS;
            float vmax = va, vmin = va;
            if (vb > vmax) vmax = vb;
            if (vb < vmin) vmin = vb;
            if (vc > vmax) vmax = vc;
            if (vc < vmin) vmin = vc;
            float v0 = (vmax + vmin) * 0.5f;

            ol_snap_theta = (int)(theta * 57.2958f);
            ol_snap_K     = (int)(current_K * 1000.0f);
            ol_snap_va    = (int)((va - v0) * 1000.0f);
            ol_snap_vb    = (int)((vb - v0) * 1000.0f);
            ol_snap_vc    = (int)((vc - v0) * 1000.0f);
            ol_snap_adc[0] = adc_raw[0];
            ol_snap_adc[1] = adc_raw[1];
            ol_snap_adc[2] = adc_raw[2];
            ol_snap_enc   = (int)enc_elec;
            ol_trace_ready = 1;
        }
    }

    /* --- 推进电角度 (锁定模式不推进) --- */
    if (!lock_mode) {
        theta += theta_step;
        if (theta >= TWO_PI) theta -= TWO_PI;
    }
}

/**
 * \brief           获取当前状态 (调试用)
 */
void openloop_get_state(float *k, float *angle, float *freq)
{
    *k     = current_K;
    *angle = theta;
    *freq  = target_freq;
}

/**
 * \brief           启动编码器对准 (锁定转子到 A 相)
 */
void openloop_start_align(float voltage)
{
    if (voltage < 0.05f) voltage = 0.2f;
    if (voltage > 0.6f)  voltage = 0.6f;

    lock_mode = 1;
    theta     = PI_OVER_2;
    target_K  = voltage;
}

/**
 * \brief           校准编码器安装偏移
 */
void openloop_calib_offset(void)
{
    /* 读取编码器机械角度, 换算电角度 (阻塞读取, 确保校准精度) */
    mt6701_data_t enc = mt6701_read_angle_blocking();
    float enc_elec = enc.angle_deg * (float)POLE_PAIRS;

    /* 归一化到 0~360° (防 NaN 死循环) */
    {
        int _loop = 0;
        while (enc_elec >= 360.0f && ++_loop < 100) enc_elec -= 360.0f;
        if (_loop >= 100) enc_elec = 0.0f;
        _loop = 0;
        while (enc_elec < 0.0f && ++_loop < 100) enc_elec += 360.0f;
        if (_loop >= 100) enc_elec = 0.0f;
    }

    /*
     * 对齐时 openloop θ=90° → va=sin90=K → A相磁场锁定转子.
     * 转子 d 轴对齐到 A 相 (0° 机械), FOC 定义此时 theta_elec = 0.
     * offset = 0° - encoder_reading, 使 Park 变换的 d/q 轴与物理一致.
     */
    encoder_offset = 0.0f - (enc_elec * TWO_PI / 360.0f);

    /* 归一到 [-π, π] */
    while (encoder_offset >  3.141592654f) encoder_offset -= TWO_PI;
    while (encoder_offset < -3.141592654f) encoder_offset += TWO_PI;

    /* 退出锁定, 停止输出 */
    lock_mode  = 0;
    target_K   = 0.0f;

    int off_deg = (int)(encoder_offset * 57.2958f);
    printf("offset calib: enc=%d deg -> offset=%d deg\r\n"
           "  copy: #define ENCODER_OFFSET_DEG  %d\r\n",
           (int)enc_elec, off_deg, off_deg);
}

/**
 * \brief           获取编码器偏移
 */
float openloop_get_offset(void)
{
    return encoder_offset;
}

/**
 * \brief           切换 SVPWM/SPWM 模式
 */
void openloop_set_svpwm(uint8_t enable)
{
    svpwm_enable = enable ? 1 : 0;
}

void openloop_get_voltages(float *va, float *vb, float *vc)
{
    *va = volt_a;
    *vb = volt_b;
    *vc = volt_c;
}

/**
 * \brief           Trace 快照打印 (主循环调用, 不在 20kHz 路径)
 */
void openloop_trace_poll(void)
{
    if (!ol_trace_ready) return;
    ol_trace_ready = 0;
    printf("O:%d,%d,%d,%d,%d,%u,%u,%u,%d\r\n",
           ol_snap_theta, ol_snap_K,
           ol_snap_va, ol_snap_vb, ol_snap_vc,
           ol_snap_adc[0], ol_snap_adc[1], ol_snap_adc[2],
           ol_snap_enc);
}