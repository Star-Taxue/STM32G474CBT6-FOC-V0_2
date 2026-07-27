/**
 * @file    svpwm.c
 * @brief   SVPWM 公共函数: 三次谐波注入 + CMP 映射 + HRTIM 寄存器写入
 * @note    被 app_foc.c 和 app_openloop.c 共用.
 *          物理接线: TimerB→A相, TimerA→B相, TimerD→C相
 *
 *          HRTIM 配置为 UpDown 中心对齐模式 + 死区互补:
 *          - 计数到 CMP1 时输出设为高电平 (上坡)
 *          - 下坡对称点自动翻转为低电平 (中心对称)
 *          - 死区自动插入, 只需设置 CMP1 即可
 */

/* Includes ------------------------------------------------------------------*/
#include "svpwm.h"

/**
 * \brief           SVPWM 三次谐波注入 + 写入 HRTIM CMP1 寄存器
 * \param[in]       va, vb, vc  三相归一化电压 (-1.0 ~ 1.0)
 * \param[in]       enable_svpwm  1=SVPWM马鞍波, 0=纯正弦(仅开环使用)
 *
 * \note            物理接线:
 *                    Timer B (PA10/PA11) → 电机 A 相
 *                    Timer A (PA8/PA9)   → 电机 B 相
 *                    Timer D (PB14/PB15) → 电机 C 相
 *
 *                  UpDown 中心对齐模式 CMP 映射:
 *                    CMP1 = CMP_CENTER - CMP_AMPLITUDE * v
 *                    v =  0 → CMP1 = 15000 (50% duty)
 *                    v = +1 → CMP1 =  1500 (~95% duty)
 *                    v = -1 → CMP1 = 28500 (~5% duty)
 *
 *                  该模式下死区硬件自动处理互补输出,
 *                  CMP3 无关联输出事件, 仅需写 CMP1.
 */
void svpwm_write(float va, float vb, float vc, int enable_svpwm)
{
    /* ---- SVPWM 三次谐波注入 (零序分量), 提升 ~15% 电压利用率 ---- */
    if (enable_svpwm) {
        float vmax = va, vmin = va;
        if (vb > vmax) vmax = vb;
        if (vb < vmin) vmin = vb;
        if (vc > vmax) vmax = vc;
        if (vc < vmin) vmin = vc;
        float v0 = (vmax + vmin) * 0.5f;
        va -= v0;
        vb -= v0;
        vc -= v0;
    }

    /*
     * UpDown 中心对齐模式: CMP1 = CMP_CENTER - CMP_AMPLITUDE * v
     * v =  0 → CMP1 = CENTER (50% duty, 三相线电压为零)
     * v = +1 → CMP1 = CENTER - AMPLITUDE (最大正占空比)
     * v = -1 → CMP1 = CENTER + AMPLITUDE (最小占空比)
     */
    int32_t cmp_a = (int32_t)(CMP_CENTER - CMP_AMPLITUDE * va); /* A相 → TimerB */
    int32_t cmp_b = (int32_t)(CMP_CENTER - CMP_AMPLITUDE * vb); /* B相 → TimerA */
    int32_t cmp_c = (int32_t)(CMP_CENTER - CMP_AMPLITUDE * vc); /* C相 → TimerD */

    /* 死区安全余量钳位 */
    if (cmp_a < (int32_t)CMP_MIN) cmp_a = CMP_MIN;
    if (cmp_a > (int32_t)CMP_MAX) cmp_a = CMP_MAX;
    if (cmp_b < (int32_t)CMP_MIN) cmp_b = CMP_MIN;
    if (cmp_b > (int32_t)CMP_MAX) cmp_b = CMP_MAX;
    if (cmp_c < (int32_t)CMP_MIN) cmp_c = CMP_MIN;
    if (cmp_c > (int32_t)CMP_MAX) cmp_c = CMP_MAX;

    /*
     * 写入 CMP1xR (仅 CMP1, 不写 CMP3):
     * HRTIM 中心对齐 + 死区模式下, 硬件根据 CMP1 自动生成
     * 互补输出和死区, CMP3 无关联输出事件无需写入.
     */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = (uint16_t)cmp_a;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = (uint16_t)cmp_b;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (uint16_t)cmp_c;
}
