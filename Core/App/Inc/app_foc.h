/**
  ******************************************************************************
  * @file           : app_foc.h
  * @brief          : 闭环 FOC 矢量控制模块
  * @note           : 包含 PI 控制器、Clarke/Park/逆Park 变换、SVPWM 输出.
  *                   电流环 20kHz (curr_sense_data_ready 上下文),
  *                   速度环 1kHz (20kHz/FOC_SPEED_LOOP_DIV 降频).
  *                   速度模式含斜坡软启动 (FOC_SPEED_RAMP).
  ******************************************************************************
  */

#ifndef __APP_FOC_H
#define __APP_FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  PI 控制器数据结构                                                          */
/* -------------------------------------------------------------------------- */

/**
 * \brief           PI 控制器状态
 * \note            Ki 使用连续域单位, 内部自动乘 Ts 转换为离散增量
 */
typedef struct {
    float Kp;           /**< 比例增益                    */
    float Ki;           /**< 积分增益 (连续域)            */
    float Ts;           /**< 采样周期 (秒)               */
    float integral;     /**< 积分累加器                   */
    float output;       /**< 最新输出值 (诊断用)          */
    float out_max;      /**< 输出上限                     */
    float out_min;      /**< 输出下限                     */
} foc_pi_t;

/* -------------------------------------------------------------------------- */
/*  控制模式 / 状态枚举                                                        */
/* -------------------------------------------------------------------------- */

/** FOC 控制模式 */
typedef enum {
    FOC_MODE_TORQUE = 0,    /**< 扭矩模式: 直接 Iq 控制           */
    FOC_MODE_SPEED  = 1     /**< 速度模式: 级联速度PI→Iq→电流PI  */
} foc_mode_t;

/** FOC 运行状态 */
typedef enum {
    FOC_STATE_IDLE    = 0,  /**< 空闲: 无输出                     */
    FOC_STATE_RUNNING = 1   /**< 运行: 闭环 FOC 激活              */
} foc_state_t;

/* -------------------------------------------------------------------------- */
/*  默认参数定义                                                               */
/* -------------------------------------------------------------------------- */

/** 采样频率 */
#define FOC_PWM_FREQ          20000.0f   /**< PWM / 电流环频率 (Hz)        */
#define FOC_TS                0.00005f   /**< 电流环采样周期 1/20000 (s)   */

/** 速度环调度 (级联在电流环内, 降频运行) */
#define FOC_SPEED_LOOP_DIV    20         /**< 速度环分频: 20kHz/20 = 1kHz  */
#define FOC_SPEED_FREQ        1000.0f    /**< 速度环频率 (Hz)              */
#define FOC_SPEED_TS          0.001f     /**< 速度环采样周期 1/1000 (s)    */
#define FOC_SPEED_RAMP        5.0f       /**< 速度斜坡: 5 RPM/step = 5000 RPM/s */

/** 编码器保护 */
#define FOC_ENC_ERR_MAX           20      /**< 连续角度失败上限 (20=0.5ms@40kHz), 超则停机 */
#define FOC_ENC_INIT_RETRY        100     /**< 启动时编码器初始化重试次数             */

/** 角度滤波器 */
#define FOC_ANGLE_DELTA_MAX       (2.0f * 3.1415926535f)  /**< 电角度跳变阈值 (rad), offset更新后允许单次大跳变 */

/** 速度估算 */
#define FOC_SPEED_RATE_LIMIT      500.0f  /**< 转速变化速率限制 (RPM/step @1kHz)        */
#define FOC_SPEED_REF_MAX         10000.0f /**< 用户转速命令上限 (RPM)                  */

/** 诊断输出 */
#define FOC_DIAG_INTERVAL         100     /**< 诊断输出间隔 (速度环tick, 100=10Hz@1kHz) */

/** SVPWM 参数 (与 hrtim.c 保持一致, UpDown 模式 PERIOD=30000) */
#define FOC_CMP_CENTER        15000U     /**< 50% 占空比 CMP 值 (duty=1-CMP1/PERIOD) */
#define FOC_CMP_AMPLITUDE     13500U     /**< 正弦幅值 (偏离中心 ±90%)  */
#define FOC_CMP_MIN            1500U     /**< 最小比较值 (~95% duty, 死区余量) */
#define FOC_CMP_MAX           28500U     /**< 最大比较值 (~5% duty, 死区余量)  */

/** 电压限制 */
#define FOC_V_MAX_NORM        0.95f      /**< 最大调制系数 (留 5% 余量)  */

/** 电流限制 */
#define FOC_IQ_MAX            5.0f       /**< Iq 最大电流 (A), 需根据电机调整 */

/** 速度估算 LPF */
#define FOC_SPEED_LPF_ALPHA   0.05f      /**< EMA 系数, 1kHz 下 fc≈8Hz     */

/** 电机参数 */
#define FOC_POLE_PAIRS        7          /**< 极对数 (与开环模块一致)     */

/** 电流 PI 默认增益 (保守值, 需根据电机 L/R 整定) */
#define FOC_DEFAULT_KP_CURRENT   0.3f   /**< 电流环 Kp  (V/A)          */
#define FOC_DEFAULT_KI_CURRENT   100.0f    /**< 电流环 Ki  (V/(A·s))      */

/** 速度 PI 默认增益 (保守值) */
#define FOC_DEFAULT_KP_SPEED     0.01f   /**< 速度环 Kp  (A/(rad/s))    */
#define FOC_DEFAULT_KI_SPEED     0.1f    /**< 速度环 Ki  (A/rad)        */

/** PI 输出钳位 (与 FOC_V_MAX_NORM 一致, 避免 anti-windup 间隙) */
#define FOC_PI_CURRENT_OUT_MAX   0.95f /**< 电流 PI 输出上限 (归一化)   */
#define FOC_PI_CURRENT_OUT_MIN  -0.95f /**< 电流 PI 输出下限 (归一化)   */

/* -------------------------------------------------------------------------- */
/*  Public API                                                                 */
/* -------------------------------------------------------------------------- */

/* --- 初始化 / 启动 / 停止 --- */

void foc_init(void);
/**< 初始化 FOC: PI 增益默认值, 从 openloop 读取 encoder offset.
     在 openloop_init() 之后调用. */

void foc_start(void);
/**< 使能 FOC 闭环: Vd=Vq=0, id_ref=iq_ref=0, 电机无力矩. */

void foc_stop(void);
/**< 禁用 FOC: Vd=Vq=0, 停止 PWM 输出电压. */

/* --- 控制模式 --- */

void foc_set_torque_mode(void);
/**< 切换到扭矩模式: 直接 Iq 参考控制. */

void foc_set_speed_mode(void);
/**< 切换到速度模式: 速度 PI → Iq 参考 → 电流 PI. */

foc_mode_t foc_get_mode(void);
/**< 返回当前控制模式. */

/* --- 参考值设置 --- */

void foc_set_iq_ref(float iq_amps);
/**< 设置 Iq 电流目标 (A). 扭矩模式下直接作为 Iq 参考.
     速度模式下被忽略 (Iq 由速度 PI 决定). */

void foc_set_speed_ref(float rpm);
/**< 设置机械转速目标 (RPM). 仅在速度模式下生效.
     存入 speed_ref_cmd, 由 foc_step() 斜坡逐步逼近. */

float foc_get_speed_ref_cmd(void);
/**< 返回用户设定的转速命令值 (RPM), 诊断用. */

/* --- 编码器 Offset 在线调整 --- */

void foc_set_encoder_offset_deg(float offset_deg);
/**< 运行时调整编码器偏移 (度, 电角度). 正负均可. */

float foc_get_encoder_offset_deg(void);
/**< 返回当前编码器偏移 (度, 电角度). */

/* --- PI 整定 (在线调参) --- */

void foc_set_current_pi(float Kp, float Ki);
/**< 同时设置 Id 和 Iq 电流环 PI 增益. */

void foc_get_current_pi(float *Kp, float *Ki);
/**< 读取电流 PI 增益. */

void foc_set_speed_pi(float Kp, float Ki);
/**< 设置速度环 PI 增益. */

void foc_get_speed_pi(float *Kp, float *Ki);
/**< 读取速度 PI 增益. */

/* -------------------------------------------------------------------------- */
/*  抗齿槽补偿 (Cogging Torque Compensation)                                   */
/* -------------------------------------------------------------------------- */

/**
 * \brief           设置齿槽谐波补偿参数
 * \param[in]       amps       补偿电流幅值 (A), 0 则禁用谐波分量
 * \param[in]       n_cycles   每电周期的齿槽波数.
 *                             12s14p 电机 = LCM(12,14)/14 = 84/14 = 6.
 *                             可用手感转动一圈数"咯噔"次数 ÷ 极对数来确定.
 * \param[in]       phase_deg  谐波相位偏移 (度)
 *
 * \note            补偿电流 = amps * sin(n_cycles * theta_elec + phase_rad)
 *                  直接叠加到 Iq 参考值上, 扭矩/速度模式均生效.
 */
void foc_cogging_set_harmonic(float amps, float n_cycles, float phase_deg);

/**
 * \brief           读取齿槽谐波参数
 */
void foc_cogging_get_harmonic(float *amps, float *n_cycles, float *phase_deg);

/**
 * \brief           设置低速额外电流增强
 * \param[in]       boost_amps      零速时的额外电流 (A)
 * \param[in]       threshold_rpm   boost 衰减到零的转速阈值 (RPM)
 *
 * \note            实际 boost = boost_amps * max(0, 1 - |speed|/threshold).
 *                  用于克服静摩擦 + 齿槽 detent, 帮助堵转后重启动.
 *                  boost_amps=0 则禁用.
 */
void foc_cogging_set_boost(float boost_amps, float threshold_rpm);

/**
 * \brief           读取低速 boost 参数
 */
void foc_cogging_get_boost(float *boost_amps, float *threshold_rpm);

/* --- 运行状态 --- */

uint8_t foc_is_enabled(void);
/**< 返回 1 如果 FOC 闭环激活. */

void foc_get_state(float *id, float *iq, float *vq, float *speed_rpm);
/**< 读取 FOC 运行状态: Id(A), Iq(A), Vq(归一化), 转速(RPM). */

void foc_get_voltages(float *va, float *vb, float *vc);
/**< 读取三相输出电压 (归一化, -1~1), 调试用. */

foc_state_t foc_get_run_state(void);
/**< 返回当前运行状态 (IDLE/RUNNING). */

uint8_t foc_get_enc_err_count(void);
/**< 返回当前连续编码器失败计数, 诊断用. */

void foc_set_trace(uint8_t on);
/**< 开启/关闭 10Hz 诊断输出 (spd,iq,id,vq,angle_ok,enc_err). */

void foc_speed_bypass(float iq);
/**< 速度环旁路: 设置固定 Iq (A), 0 则恢复速度 PI 控制. */

/* --- 快速环 (20kHz 上下文调用) --- */

void foc_step(void);
/**< 每 PWM 周期调用: 完整 FOC 算法链.
     Clarke → Park → 电流PI → 电压圆限制 → 逆Park → SVPWM → HRTIM.
     仅在 curr_sense_data_ready() 后调用. */

#ifdef __cplusplus
}
#endif

#endif /* __APP_FOC_H */
