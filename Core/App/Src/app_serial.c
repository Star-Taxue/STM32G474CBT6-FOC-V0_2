/**
  ******************************************************************************
  * @file           : app_serial.c
  * @brief          : 串口命令解析模块
  * @note           : UART RX 中断接收 → 环形缓冲 → 行解析 → 命令执行.
  *                   命令格式: "cmd [arg1] [arg2]\\r\\n"
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "app_serial.h"
#include "app_openloop.h"
#include "app_foc.h"
#include "app_current_sense.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* Private defines -----------------------------------------------------------*/

#define RX_BUF_SIZE      64    /* 环形缓冲区大小 (接收字符)         */
#define LINE_BUF_SIZE     32    /* 命令行缓冲区大小                   */
#define MAX_ARGS           5    /* 最大命令参数个数 (cmd + 4 args)    */

/* Private variables ---------------------------------------------------------*/

static uint8_t  rx_ring[RX_BUF_SIZE];  /* 环形缓冲区                  */
static volatile uint8_t rx_head;       /* 写指针 (ISR 上下文)         */
static uint8_t  rx_tail;               /* 读指针 (main 上下文)        */

static char     line_buf[LINE_BUF_SIZE]; /* 当前命令行缓冲区           */
static uint8_t  line_idx;               /* 命令行写入位置              */

static uint8_t  motor_running;          /* 电机运行标志                */

/* Private function prototypes -----------------------------------------------*/

/**
 * \brief           环形缓冲区判空
 */
static inline int ring_empty(void)
{
    return (rx_head == rx_tail);
}

/**
 * \brief           从环形缓冲区读一个字符 (非阻塞)
 * \param[out]      ch  读取的字符
 * \retval          1 成功, 0 缓冲区空
 */
static int ring_getchar(char *ch)
{
    if (ring_empty()) return 0;
    *ch = (char)rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}

/**
 * \brief           执行单条命令
 * \param[in]       cmd_line  命令行 (以 \\0 结尾)
 */
static void serial_execute(char *cmd_line)
{
    /* 跳过前导空白 */
    while (*cmd_line == ' ' || *cmd_line == '\t') cmd_line++;
    if (*cmd_line == '\0') return;  /* 空行 */

    /* 分割命令和参数 */
    char *argv[MAX_ARGS] = {NULL, NULL, NULL};
    int argc = 0;
    char *token = strtok(cmd_line, " \t\r\n");
    while (token && argc < MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " \t\r\n");
    }

    const char *cmd = argv[0];

    /* --- 命令分发 --- */
    if (strcmp(cmd, "start") == 0) {
        /* start [freq] [volt] */
        if (argc >= 2) {
            openloop_set_freq((float)atof(argv[1]));
        }
        if (argc >= 3) {
            openloop_set_voltage((float)atof(argv[2]));
        } else {
            openloop_set_voltage(0.10f);  /* 默认 10% */
        }
        motor_running = 1;
        float _f = (float)atof(argc >= 2 ? argv[1] : "10");
        float _v = (float)atof(argc >= 3 ? argv[2] : "0.1");
        printf("start freq=%d volt=%d\r\n",
               (int)_f, (int)(_v * 100.0f));

    } else if (strcmp(cmd, "stop") == 0) {
        openloop_set_voltage(0.0f);
        if (foc_is_enabled()) {
            foc_stop();
        }
        motor_running = 0;
        printf("stop\r\n");

    } else if (strcmp(cmd, "calib") == 0) {
        int samples = (argc >= 2) ? atoi(argv[1]) : 1024;
        if (samples < 64) samples = 64;
        printf("calibrating ADC offset (%d samples)...\r\n", samples);

        /* 1. 停掉开环和 FOC, K 归零 */
        openloop_set_voltage(0.0f);
        if (foc_is_enabled()) {
            foc_stop();
        }
        motor_running = 0;

        /* 2. 等待 soft-start ramp 降到 0 (K/rate=0.3/0.0003=1000步≈50ms) */
        HAL_Delay(100);

        /* 3. 强制三相 CMP=15000 (50% duty), 线电压为零, 电流归零 */
        HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = CMP_CENTER;
        HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = CMP_CENTER;
        HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = CMP_CENTER;

        /* 4. 等待绕组电流衰减 */
        HAL_Delay(200);

        /* 5. 采集零点 */
        curr_sense_calibrate((uint32_t)samples);
        uint16_t oa, ob, oc;
        curr_sense_get_offsets(&oa, &ob, &oc);
        printf("calib done: A=%u B=%u C=%u\r\n", oa, ob, oc);
        printf("  update #define in app_current_sense.h:\r\n");
        printf("  #define CURR_SENSE_OFFSET_A  %u\r\n", oa);
        printf("  #define CURR_SENSE_OFFSET_B  %u\r\n", ob);
        printf("  #define CURR_SENSE_OFFSET_C  %u\r\n", oc);

    } else if (strcmp(cmd, "f") == 0 || strcmp(cmd, "freq") == 0) {
        if (argc >= 2) {
            float f = (float)atof(argv[1]);
            openloop_set_freq(f);
            printf("freq=%d Hz\r\n", (int)f);
        } else {
            printf("usage: freq <Hz>\r\n");
        }

    } else if (strcmp(cmd, "v") == 0 || strcmp(cmd, "volt") == 0) {
        if (argc >= 2) {
            float k = (float)atof(argv[1]);
            openloop_set_voltage(k);
            printf("volt=%d\r\n", (int)(k * 100.0f));
        } else {
            printf("usage: volt <k>\r\n");
        }

    } else if (strcmp(cmd, "mode") == 0) {
        if (argc >= 2 && strcmp(argv[1], "spwm") == 0) {
            openloop_set_svpwm(0);
            printf("mode: SPWM\r\n");
        } else if (argc >= 2 && strcmp(argv[1], "svpwm") == 0) {
            openloop_set_svpwm(1);
            printf("mode: SVPWM\r\n");
        } else {
            printf("mode: spwm/svpwm\r\n");
        }

    } else if (strcmp(cmd, "align") == 0) {
        /* align [volt] — 锁定转子到 A 相, 默认 0.3 */
        float av = (argc >= 2) ? (float)atof(argv[1]) : 0.3f;
        openloop_start_align(av);
        motor_running = 0;
        printf("align volt=%d (locking... wait 1s then 'offset')\r\n",
               (int)(av * 100.0f));

    } else if (strcmp(cmd, "offset") == 0) {
        openloop_calib_offset();
        motor_running = 0;

    } else if (strcmp(cmd, "pos") == 0) {
        /* pos on/off → 位置模式, pos <deg> → 设目标 */
        if (argc >= 2 && strcmp(argv[1], "on") == 0) {
            foc_pos_enable(1);
            printf("pos mode on\r\n");
        } else if (argc >= 2 && strcmp(argv[1], "off") == 0) {
            foc_pos_enable(0);
            printf("pos mode off\r\n");
        } else if (argc >= 2) {
            float deg = (float)atof(argv[1]);
            foc_set_target(deg);
            foc_pos_enable(1);
            printf("pos target=%d deg\r\n", (int)deg);
        } else {
            printf("pos=%d deg  usage: pos on|off | pos <deg> | pos_kp <val>\r\n",
                   (int)foc_get_position());
        }

    } else if (strcmp(cmd, "pos_kp") == 0) {
        if (argc >= 2) foc_set_pos_kp((float)atof(argv[1]));
        printf("pos_kp set\r\n");

    } else if (strcmp(cmd, "pos_kd") == 0) {
        if (argc >= 2) foc_set_pos_kd((float)atof(argv[1]));
        printf("pos_kd set\r\n");

    } else if (strcmp(cmd, "pos_ki") == 0) {
        if (argc >= 2) foc_set_pos_ki((float)atof(argv[1]));
        printf("pos_ki set\r\n");

    } else if (strcmp(cmd, "test_theta") == 0) {
        /* 固定电角度测试: test_theta 90 → θ=90°, test_theta -1 → 退出 */
        if (argc >= 2) {
            float deg = (float)atof(argv[1]);
            foc_test_theta(deg);
            if (deg < 0.0f) {
                printf("test_theta: off (encoder mode)\r\n");
            } else {
                printf("test_theta: locked at %d deg\r\n", (int)deg);
            }
        } else {
            printf("usage: test_theta <deg>  (-1 to disable)\r\n");
        }

    } else if (strcmp(cmd, "foc") == 0) {
        if (argc >= 2 && strcmp(argv[1], "on") == 0) {
            foc_start();
            motor_running = 1;
        } else if (argc >= 2 && strcmp(argv[1], "off") == 0) {
            foc_stop();
            motor_running = 0;
        } else {
            printf("usage: foc on|off\r\n");
        }

    } else if (strcmp(cmd, "torque") == 0) {
        if (argc >= 2) {
            float iq = (float)atof(argv[1]);
            foc_set_iq_ref(iq);
            printf("torque Iq=%d mA\r\n", (int)(iq * 1000.0f));
        } else {
            printf("usage: torque <A> (Iq ref=%d mA)\r\n",
                   (int)(foc_get_iq_ref() * 1000.0f));
        }

    } else if (strncmp(cmd, "kp_i:", 5) == 0) {
        float kp = (float)atof(cmd + 5);
        foc_set_current_kp(kp);
        printf("Kp_i=%d\r\n", (int)(kp * 1000.0f));

    } else if (strncmp(cmd, "ki_i:", 5) == 0) {
        float ki = (float)atof(cmd + 5);
        foc_set_current_ki(ki);
        printf("Ki_i=%d\r\n", (int)(ki * 1000.0f));

    } else if (strcmp(cmd, "pid") == 0) {
        if (argc >= 3) {
            float kp = (float)atof(argv[1]);
            float ki = (float)atof(argv[2]);
            foc_set_current_kp(kp);
            foc_set_current_ki(ki);
            printf("pid: Kp=%d Ki=%d\r\n",
                   (int)(kp * 1000.0f), (int)(ki * 1000.0f));
        } else {
            float kp, ki;
            foc_get_current_pi(&kp, &ki);
            printf("pid current: Kp=%d Ki=%d\r\n",
                   (int)(kp * 1000.0f), (int)(ki * 1000.0f));
            printf("usage: pid <Kp> <Ki>\r\n");
        }

    } else if (strcmp(cmd, "?") == 0 || strcmp(cmd, "info") == 0) {
        if (foc_is_enabled()) {
            printf("FOC: Iq ref=%d mA\r\n",
                   (int)(foc_get_iq_ref() * 1000.0f));
        } else {
            float k, theta, freq;
            openloop_get_state(&k, &theta, &freq);
            printf("state:%s K:%d F:%d\r\n",
                   motor_running ? "RUN" : "STOP",
                   (int)(k * 100.0f), (int)freq);
        }

    } else if (strcmp(cmd, "trace") == 0) {
        extern uint8_t g_trace_enabled;
        if (argc >= 2 && strcmp(argv[1], "on") == 0) {
            g_trace_enabled = 1;
            printf("trace on\r\n");
        } else if (argc >= 2 && strcmp(argv[1], "off") == 0) {
            g_trace_enabled = 0;
            printf("trace off\r\n");
        } else {
            printf("usage: trace on|off\r\n");
        }

    } else if (strcmp(cmd, "diag") == 0) {
        if (foc_is_enabled()) {
            int iqr, iq, id, vd, vq, th, ia, ib, pos;
            foc_trace_get(&iqr, &iq, &id, &vd, &vq, &th, &ia, &ib, &pos);
            printf("diag: iq_ref=%d iq=%d id=%d vd=%d vq=%d th=%d ia=%d ib=%d pos=%d\r\n",
                   iqr, iq, id, vd, vq, th, ia, ib, pos);
        } else {
            float k, theta, freq;
            openloop_get_state(&k, &theta, &freq);
            printf("state:%s K:%d F:%d\r\n",
                   motor_running ? "RUN" : "STOP",
                   (int)(k * 100.0f), (int)freq);
        }

    } else {
        printf("unknown: %s\r\n", cmd);
        printf("cmds: start/stop/f/v/align/offset/foc/torque/speed/mode/pid/cogging/?\r\n");
    }
}

/* Public API ----------------------------------------------------------------*/

/**
 * \brief           初始化串口命令解析
 * \note            启动 UART1 单字节 RX 中断接收.
 *                  在 MX_USART1_UART_Init() 之后调用.
 */
void serial_init(void)
{
    rx_head  = 0;
    rx_tail  = 0;
    line_idx = 0;
    motor_running = 0;

    /* 启动单字节中断接收 */
    HAL_UART_Receive_IT(&huart1, &rx_ring[rx_head], 1);
}

/**
 * \brief           轮询处理串口命令
 * \note            在 main 循环中定期调用 (如每 10ms).
 *                  从环形缓冲区读取字符, 遇换行则执行命令.
 */
void serial_poll(void)
{
    char ch;

    while (ring_getchar(&ch)) {
        /* 忽略回车, 换行触发命令执行 */
        if (ch == '\r') continue;

        if (ch == '\n') {
            line_buf[line_idx] = '\0';
            serial_execute(line_buf);
            line_idx = 0;
        } else if (line_idx < LINE_BUF_SIZE - 1) {
            line_buf[line_idx++] = ch;
        }
        /* 缓冲区满则丢弃, 防止溢出 */
    }
}

/**
 * \brief           UART RX 接收完成回调 (HAL 弱定义覆盖)
 * \note            由 USART1_IRQHandler -> HAL_UART_IRQHandler 调用.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* 推进写指针 */
        uint8_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) {        /* 未满 */
            rx_head = next;
        }
        /* 丢失字符时不做特殊处理 (环形覆盖风险已规避) */

        /* 重新启动单字节中断接收 */
        HAL_UART_Receive_IT(&huart1, &rx_ring[rx_head], 1);
    }
}

