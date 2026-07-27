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

void mt6701_init(void);
mt6701_data_t mt6701_read_angle(void);

#endif /* __MT6701_H */
