/**
 * @file    app_uart.c
 * @brief   应用层 UART 通信模块
 * @note    用于调试数据输出和虚拟示波器通道
 */

#include "app_uart.h"
#include <stdio.h>

/**
 * @brief  发送 4 通道数据 (虚拟示波器格式)
 * @note   嵌入式 printf 通常不支持 %f, 使用整数转换或启用浮点链接选项
 *         在 CMakeLists.txt 中添加: target_link_options(... PRIVATE -u _printf_float)
 */
void uart_send_channels(float ch1, float ch2, float ch3, float ch4)
{
    printf("channels: %d,%d,%d,%d\r\n",
           (int)(ch1 * 1000.0f), (int)(ch2 * 1000.0f),
           (int)(ch3 * 1000.0f), (int)(ch4 * 1000.0f));
}
