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
        } else if (argc >= 2 && strcmp(argv[1], "torque") == 0) {
            foc_set_torque_mode();
            printf("mode: torque (FOC)\r\n");
        } else if (argc >= 2 && strcmp(argv[1], "speed") == 0) {
            foc_set_speed_mode();
            printf("mode: speed (FOC)\r\n");
        } else {
            printf("mode: spwm/svpwm (openloop) | torque/speed (FOC)\r\n");
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

    } else if (strcmp(cmd, "eoffset") == 0) {
        /* 在线调整编码器 offset */
        if (argc >= 2) {
            float deg = (float)atof(argv[1]);
            foc_set_encoder_offset_deg(deg);
        } else {
            printf("encoder offset = %d deg\r\n",
                   (int)foc_get_encoder_offset_deg());
            printf("usage: eoffset <deg>\r\n");
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
            printf("usage: torque <A>\r\n");
        }

    } else if (strcmp(cmd, "speed") == 0) {
        if (argc >= 2) {
            float rpm = (float)atof(argv[1]);
            foc_set_speed_ref(rpm);
            printf("speed cmd=%d RPM (ramp %d RPM/ms)\r\n",
                   (int)rpm, (int)FOC_SPEED_RAMP);
        } else {
            printf("speed cmd=%d RPM\r\n", (int)foc_get_speed_ref_cmd());
        }

    } else if (strncmp(cmd, "kp_i:", 5) == 0) {
        float kp = (float)atof(cmd + 5);
        float _kp, ki;
        foc_get_current_pi(&_kp, &ki);
        foc_set_current_pi(kp, ki);

    } else if (strncmp(cmd, "ki_i:", 5) == 0) {
        float ki = (float)atof(cmd + 5);
        float kp, _ki;
        foc_get_current_pi(&kp, &_ki);
        foc_set_current_pi(kp, ki);

    } else if (strncmp(cmd, "kp_s:", 5) == 0) {
        float kp = (float)atof(cmd + 5);
        float _kp, ki;
        foc_get_speed_pi(&_kp, &ki);
        foc_set_speed_pi(kp, ki);

    } else if (strncmp(cmd, "ki_s:", 5) == 0) {
        float ki = (float)atof(cmd + 5);
        float kp, _ki;
        foc_get_speed_pi(&kp, &_ki);
        foc_set_speed_pi(kp, ki);

    } else if (strcmp(cmd, "pid") == 0) {
        if (argc >= 2 && strcmp(argv[1], "current") == 0 && argc >= 4) {
            float kp = (float)atof(argv[2]);
            float ki = (float)atof(argv[3]);
            foc_set_current_pi(kp, ki);
        } else if (argc >= 2 && strcmp(argv[1], "speed") == 0 && argc >= 4) {
            float kp = (float)atof(argv[2]);
            float ki = (float)atof(argv[3]);
            foc_set_speed_pi(kp, ki);
        } else {
            /* 打印当前 PI 参数 */
            float cp_kp, cp_ki, sp_kp, sp_ki;
            foc_get_current_pi(&cp_kp, &cp_ki);
            foc_get_speed_pi(&sp_kp, &sp_ki);
            printf("pid current: Kp=%d Ki=%d\r\n",
                   (int)(cp_kp * 1000.0f), (int)(cp_ki * 1000.0f));
            printf("pid speed:   Kp=%d Ki=%d\r\n",
                   (int)(sp_kp * 1000.0f), (int)(sp_ki * 1000.0f));
            printf("usage: pid current|speed <Kp> <Ki>\r\n");
        }

    } else if (strcmp(cmd, "cogging") == 0 || strcmp(cmd, "cog") == 0) {
        if (argc >= 2 && strcmp(argv[1], "off") == 0) {
            foc_cogging_set_harmonic(0.0f, 6.0f, 0.0f);
            foc_cogging_set_boost(0.0f, 200.0f);
            printf("cogging: off\r\n");
        } else if (argc >= 4) {
            /* cogging <amps_A> <n_cycles> <phase_deg> [boost_A] [boost_rpm] */
            float amps = (float)atof(argv[1]);
            float n    = (float)atof(argv[2]);
            float ph   = (float)atof(argv[3]);
            foc_cogging_set_harmonic(amps, n, ph);

            if (argc >= 6) {
                float bst = (float)atof(argv[4]);
                float thr = (float)atof(argv[5]);
                foc_cogging_set_boost(bst, thr);
            }
        } else {
            /* 显示当前配置 */
            float amps, n, ph;
            foc_cogging_get_harmonic(&amps, &n, &ph);
            float bst, thr;
            foc_cogging_get_boost(&bst, &thr);

            printf("cogging: %s\r\n", (amps == 0.0f && bst == 0.0f) ? "off" : "active");
            printf("  harmonic: A=%d mA n=%d phase=%d deg\r\n",
                   (int)(amps * 1000.0f), (int)n, (int)ph);
            printf("  boost:    %d mA threshold=%d RPM\r\n",
                   (int)(bst * 1000.0f), (int)thr);
            printf("usage: cogging off | <amps_A> <n> <phase_deg> [boost_A] [boost_rpm]\r\n");
            printf("  eg:  cogging 0.1 6 0         (0.1A harmonic only)\r\n");
            printf("  eg:  cogging 0.1 6 0 1.0 300 (harmonic + 1A boost below 300RPM)\r\n");
        }

    } else if (strcmp(cmd, "?") == 0 || strcmp(cmd, "info") == 0) {
        if (foc_is_enabled()) {
            float id, iq, vq, speed;
            foc_get_state(&id, &iq, &vq, &speed);
            if (foc_get_mode() == FOC_MODE_SPEED) {
                printf("FOC:%s mode=S Id:%d Iq:%d Vq:%d spd:%d cmd:%d enc:%d\r\n",
                       motor_running ? "RUN" : "STOP",
                       (int)(id * 1000.0f), (int)(iq * 1000.0f),
                       (int)(vq * 1000.0f), (int)speed,
                       (int)foc_get_speed_ref_cmd(),
                       foc_get_enc_err_count());
            } else {
                printf("FOC:%s mode=T Id:%d Iq:%d Vq:%d spd:%d enc:%d\r\n",
                       motor_running ? "RUN" : "STOP",
                       (int)(id * 1000.0f), (int)(iq * 1000.0f),
                       (int)(vq * 1000.0f), (int)speed,
                       foc_get_enc_err_count());
            }
        } else {
            float k, theta, freq;
            openloop_get_state(&k, &theta, &freq);
            printf("state:%s K:%d F:%d\r\n",
                   motor_running ? "RUN" : "STOP",
                   (int)(k * 100.0f), (int)freq);
        }

    } else if (strcmp(cmd, "trace") == 0) {
        if (argc >= 2 && strcmp(argv[1], "on") == 0) {
            foc_set_trace(1);
        } else if (argc >= 2 && strcmp(argv[1], "off") == 0) {
            foc_set_trace(0);
        } else {
            printf("usage: trace on|off  (10Hz diag: spd,iq,id,vq,ok,enc_err)\r\n");
        }

    } else if (strcmp(cmd, "bypass") == 0) {
        /* 速度环旁路: bypass 0.5 → Iq=0.5A 固定, 绕过速度PI */
        if (argc >= 2) {
            float iq = (float)atof(argv[1]);
            foc_speed_bypass(iq);
        } else {
            foc_speed_bypass(0.0f);
            printf("bypass: speed PI restored\r\n");
        }

    } else if (strcmp(cmd, "odebug") == 0) {
        if (argc >= 2 && strcmp(argv[1], "off") == 0) {
            openloop_set_debug(0);
            printf("odebug off\r\n");
        } else {
            openloop_set_debug(1);
            printf("odebug on (default)\r\n");
        }

    } else if (strcmp(cmd, "diag") == 0) {
        if (foc_is_enabled()) {
            float id, iq, vq, speed;
            foc_get_state(&id, &iq, &vq, &speed);
            printf("diag: spd=%d Iq=%d Id=%d Vq=%d ok=%d enc=%d state=%d\r\n",
                   (int)speed,
                   (int)(iq * 1000.0f), (int)(id * 1000.0f),
                   (int)(vq * 1000.0f),
                   foc_get_enc_err_count() == 0 ? 1 : 0,
                   foc_get_enc_err_count(),
                   foc_get_run_state());
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

