#ifndef __MT6701_H
#define __MT6701_H

#include <stdint.h>

/* MT6701 磁编码器数据结构 */
typedef struct {
    uint16_t raw_angle;      /* 14位原始角度值 (0-16383)        */
    float    angle_deg;      /* 转换后的角度 (0-360°)           */
    uint8_t  status;         /* 4位状态标志                     */
    uint8_t  crc;            /* 6位CRC校验值                    */
    uint8_t  crc_valid;      /* CRC校验结果: 0=失败, 1=通过    */
} mt6701_data_t;

/** MT6701 角度转换: 360° / 16384 = 0.02197265625 */
#define MT6701_ANGLE_SCALE  0.02197265625f

/**
 * @brief  MT6701 CRC-6 多项式 (x^6 + x + 1)
 */
#define MT6701_CRC6_POLY  0x03U

/* -------------------------------------------------------------------------- */
/*  Public API                                                                 */
/* -------------------------------------------------------------------------- */

void mt6701_init(void);
/**< 初始化编码器: DWT 周期计数器, CSN 拉高, 等待上电稳定.
     完成后执行一次阻塞读取, 填充缓存供后续非阻塞读取使用. */

void mt6701_trigger_read(void);
/**< 启动一次非阻塞 DMA SPI 读取. 若上次传输未完成则跳过.
     应在每个控制周期结束时调用, 保证下一周期的数据已就绪.
     调用后 ~9μs DMA 完成, 结果存入缓存, 由 mt6701_read_angle() 读取. */

mt6701_data_t mt6701_read_angle(void);
/**< 从内存直接读取最近一次编码器角度 (非阻塞, 无 SPI 访问).
     数据由 mt6701_trigger_read() 触发的 DMA 完成中断刷新.
     缓存初始值由 mt6701_init() 填充.
     @retval 最近一次的有效数据. 若 CRC 连续失败则 crc_valid=0. */

mt6701_data_t mt6701_read_angle_blocking(void);
/**< 阻塞式 SPI 读取 (含最多 3 次 CRC 重试).
     用于校准、诊断等非实时场合. 不应在 20kHz 控制环中调用. */

uint8_t mt6701_is_data_fresh(void);
/**< 检查是否有新数据就绪 (自上次 mt6701_read_angle() 后).
     @retval 1=有新数据, 0=自上次读取后暂无更新. */

uint8_t mt6701_is_busy(void);
/**< 查询 SPI+DMA 是否正忙 (传输进行中).
     @retval 1=忙, 0=空闲. */

#endif /* __MT6701_H */
