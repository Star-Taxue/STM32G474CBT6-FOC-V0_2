#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * \brief           初始化串口命令解析 (启动 UART RX 中断接收)
 */
void serial_init(void);

/**
 * \brief           轮询串口命令 (非阻塞, 在 main 循环中定期调用)
 * \note            收到完整行后解析并执行命令, 通过回调控制电机.
 */
void serial_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SERIAL_H */
