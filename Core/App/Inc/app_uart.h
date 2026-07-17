#ifndef __APP_UART_H
#define __APP_UART_H

#include <stdint.h>

/**
 * @brief  通过 UART 发送 4 通道浮点数据 (用于虚拟示波器调试)
 * @param  ch1..ch4  4 个通道的浮点值
 * @note   格式: "channels: %.2f,%.2f,%.2f,%.2f\n"
 */
void uart_send_channels(float ch1, float ch2, float ch3, float ch4);

#endif /* __APP_UART_H */
