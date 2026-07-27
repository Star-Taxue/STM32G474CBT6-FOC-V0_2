/**
 * @file    svpwm.h
 * @brief   SVPWM 公共函数声明: 三次谐波注入 + CMP 映射 + HRTIM 寄存器写入
 * @note    被 app_foc.c 和 app_openloop.c 共用.
 *          物理接线: TimerB→A相, TimerA→B相, TimerD→C相
 */

#ifndef __SVPWM_H
#define __SVPWM_H

#include "main.h"
#include "hrtim.h"
#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief           SVPWM 三次谐波注入 + 写入 HRTIM CMP1 寄存器
 * \param[in]       va, vb, vc  三相归一化电压 (-1.0 ~ 1.0)
 * \param[in]       enable_svpwm  1=SVPWM马鞍波, 0=纯正弦(仅开环使用)
 *
 * \note            所有调用者共享此函数, 避免 CMP 映射和寄存器写入的重复代码.
 *                  物理接线:
 *                    Timer B (PA10/PA11) → 电机 A 相
 *                    Timer A (PA8/PA9)   → 电机 B 相
 *                    Timer D (PB14/PB15) → 电机 C 相
 *
 *                  HRTIM UpDown 中心对齐模式:
 *                    上坡计数到 CMP1 → 输出高电平
 *                    下坡对称点 → 自动翻转为低电平
 *                    死区硬件自动插入互补输出
 *                    仅需写 CMP1xR, 不写 CMP3xR
 */
void svpwm_write(float va, float vb, float vc, int enable_svpwm);

#ifdef __cplusplus
}
#endif

#endif /* __SVPWM_H */
