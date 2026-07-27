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
#define ENCODER_OFFSET_DEG  100   /* 校准值: 2026-07-23 */
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
static uint8_t  ol_debug;        /* 开环调试输出开关                     */
static uint32_t ol_step_cnt;     /* step 计数 (验证 20kHz 时序)          */
static uint32_t ol_debug_period; /* 调试输出间隔 (step 数)               */
static volatile uint8_t ol_snapshot_ready; /* 快照就绪, 主循环打印        */
static float    ol_snap_theta;   /* 快照: 电角度                         */
static float    ol_snap_K;       /* 快照: 调制系数                       */
static float    ol_snap_va;      /* 快照: Va                            */
static float    ol_snap_vb;      /* 快照: Vb                            */
static float    ol_snap_vc;      /* 快照: Vc                            */
static uint16_t ol_snap_adc[3];  /* 快照: ADC 原始值                     */
static float    ol_snap_enc;      /* 快照: 编码器电角度 (deg)              */

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
    ol_debug       = 1;   /* 默认开启开环调试 */
    ol_step_cnt    = 0;
    ol_debug_period = 300; /* ~200ms @ 20kHz, 非整数倍电周期打破 aliasing */
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

    /* K ≈ 0 时不更新占空比 (减少寄存器写入) */
    if (current_K < 0.0005f) {
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

    /* --- 调试快照 (5Hz, 不阻塞实时路径) --- */
    if (ol_debug) {
        ol_step_cnt++;
        if (ol_step_cnt >= ol_debug_period) {
            ol_step_cnt = 0;
            ol_snap_theta = theta;
            ol_snap_K     = current_K;
            /* 计算 SVPWM 注入后的马鞍波 (与 svpwm_write 内部逻辑一致) */
            {
                float vmax = volt_a, vmin = volt_a;
                if (volt_b > vmax) vmax = volt_b;
                if (volt_b < vmin) vmin = volt_b;
                if (volt_c > vmax) vmax = volt_c;
                if (volt_c < vmin) vmin = volt_c;
                float v0 = (vmax + vmin) * 0.5f;
                ol_snap_va = volt_a - v0;
                ol_snap_vb = volt_b - v0;
                ol_snap_vc = volt_c - v0;
            }
            curr_sense_get_raw_adc(&ol_snap_adc[0], &ol_snap_adc[1], &ol_snap_adc[2]);
            ol_snap_enc = mt6701_read_angle().angle_deg * (float)POLE_PAIRS;
            ol_snapshot_ready = 1;
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
    /* 读取编码器机械角度, 换算电角度 */
    mt6701_data_t enc = mt6701_read_angle();
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
 * \brief           开/关 开环 5Hz 调试输出
 * \param[in]       on  1=开启, 0=关闭
 */
void openloop_set_debug(uint8_t on)
{
    ol_debug = on ? 1 : 0;
    if (on) {
        ol_step_cnt = 0;
        printf("openloop debug ON (%lu steps = %d ms)\r\n",
               ol_debug_period, (int)(ol_debug_period * 50 / 1000));
    }
}

/**
 * \brief           开环调试快照打印 (主循环中调用, 非实时)
 * \note            由 openloop_step() 在当前 PWM 周期捕获快照,
 *                  主循环空闲时打印, 不阻塞 20kHz 控制路径.
 */
void openloop_debug_poll(void)
{
    if (!ol_snapshot_ready) return;
    ol_snapshot_ready = 0;

    /* 编码器电角度归一化到 0~360° */
    float enc = ol_snap_enc;
    {
        int _n = 0;
        while (enc >= 360.0f && ++_n < 100) enc -= 360.0f;
        _n = 0;
        while (enc < 0.0f && ++_n < 100) enc += 360.0f;
    }

    printf("OL:cnt=%lu th=%d K=%d, %d, %d, %d, %u, %u, %u, %d\r\n",
           ol_debug_period,
           (int)(ol_snap_theta * 57.2958f),
           (int)(ol_snap_K * 1000.0f),
           (int)(ol_snap_va * 1000.0f),
           (int)(ol_snap_vb * 1000.0f),
           (int)(ol_snap_vc * 1000.0f),
           ol_snap_adc[0], ol_snap_adc[1], ol_snap_adc[2],
           (int)enc);
}

