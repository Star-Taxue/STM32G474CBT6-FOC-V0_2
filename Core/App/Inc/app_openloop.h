#ifndef __APP_OPENLOOP_H
#define __APP_OPENLOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * \brief           初始化开环电压驱动
 */
void openloop_init(void);

/**
 * \brief           设置目标电压幅值
 * \param[in]       k  调制系数 (0.0 ~ 1.0)
 */
void openloop_set_voltage(float k);

/**
 * \brief           设置目标电频率
 * \param[in]       freq_hz  电频率 (Hz)
 */
void openloop_set_freq(float freq_hz);

/**
 * \brief           每 PWM 周期调用: 更新角度和三相占空比
 */
void openloop_step(void);

/**
 * \brief           获取当前状态
 * \param[out]      k      当前调制系数
 * \param[out]      theta  当前电角度 (rad)
 * \param[out]      freq   目标频率 (Hz)
 */
void openloop_get_state(float *k, float *theta, float *freq);

/**
 * \brief           启动编码器对准 (锁定转子到 A 相电角度 90°)
 * \param[in]       voltage  对准电压 (调制系数, 建议 0.2~0.4)
 * \note            注入固定电压矢量, 转子被磁场吸合锁定.
 *                  保持 ~1s 后再调用 openloop_calib_offset().
 */
void openloop_start_align(float voltage);

/**
 * \brief           校准编码器安装偏移
 * \note            在 align 完成 (转子吸合稳定) 后调用.
 *                  读取编码器角度, 计算偏差并保存, 自动退出锁定模式.
 */
void openloop_calib_offset(void);

/**
 * \brief           获取编码器偏移值
 * \return          偏移角 (rad, 电角度)
 */
float openloop_get_offset(void);

/**
 * \brief           设置调制模式
 * \param[in]       svpwm  0=SPWM 纯正弦, 1=SVPWM 马鞍波
 */
void openloop_set_svpwm(uint8_t svpwm);

/**
 * \brief           获取当前输出电压 (调试用, 查看马鞍波形)
 * \param[out]      va, vb, vc  三相电压 (-1.0 ~ 1.0, 归一化)
 */
void openloop_get_voltages(float *va, float *vb, float *vc);

void openloop_set_debug(uint8_t on);
void openloop_debug_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_OPENLOOP_H */
