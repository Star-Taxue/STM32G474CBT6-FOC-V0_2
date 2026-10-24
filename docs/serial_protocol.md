# 串口与 VOFA+ 协议

串口使用 ASCII 行协议，每条命令以 `\r\n` 结束。VOFA+ 控件可直接发送下面的键值格式。

## 功能开关

使用一个十六进制数控制功能位：

| 位 | 掩码 | 功能 |
|---:|---:|---|
| 0 | `0x01` | FOC 使能 |
| 1 | `0x02` | 速度环 |
| 2 | `0x04` | 位置环 |
| 3 | `0x08` | Trace 输出 |

命令：

```text
sw:0x0B
```

`0x0B` 表示 FOC、速度环和 Trace 开启，位置环关闭。开关状态回显为 `sw:0xNN`。

开环仍使用独立命令，不由 `sw` 位掩码启动：

```text
start 10 0.1
stop
```

## VOFA+ 键值命令

```text
kp:0.390          /* 位置环 Kp */
kd:1.000          /* 位置环 Kd */
ki:0.000          /* 位置环 Ki */
speed_kp:0.010    /* 速度环 Kp */
speed_ki:0.100    /* 速度环 Ki */
speed:30          /* 速度给定 RPM, 自动打开速度环 */
pos:90            /* 位置目标角度, 自动打开位置环 */
iq:0.200          /* Iq 给定 A */
```

参数回显使用固定三位小数，例如：

```text
kp:0.390
speed_kp:0.010
```

`pid:` 不再用于位置环参数；位置环比例参数使用 `kp:`。旧的空格命令仍可用于人工调试，例如 `speed_pid 0.01 0.1`。

## Trace 输出

Trace 行格式为：

```text
F:<field1>,<field2>,...
```

发送字段在 `Core/App/Inc/motor_config.h` 中控制。默认顺序为：

```text
iq_ref, iq, id, theta, pos, speed_ref, speed_est
```

对应配置宏为 `TRACE_SEND_IQ_REF`、`TRACE_SEND_IQ`、`TRACE_SEND_ID`、`TRACE_SEND_VD`、`TRACE_SEND_VQ`、`TRACE_SEND_THETA`、`TRACE_SEND_IA`、`TRACE_SEND_IB`、`TRACE_SEND_POS`、`TRACE_SEND_SPEED_REF` 和 `TRACE_SEND_SPEED_EST`。

将不需要的宏改为 `0` 或注释掉即可减少 ISR 快照计算和 UART 输出；修改后需要重新构建、烧录。
