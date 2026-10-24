# STM32G474CBT6 FOC 电机控制固件

基于 STM32G474CBT6 的磁场定向控制(FOC)固件,三环级联(位置/速度/电流),支持 MT6701 磁编码器、串口命令行调参与 200Hz 数据 trace。

## 硬件平台

| 项 | 说明 |
|---|---|
| MCU | STM32G474CBT6,128K Flash / 128K RAM |
| 时钟 | HSI 16MHz → PLL(M=DIV4, N=75, P=DIV2) → **SYSCLK 150MHz** |
| 功率级 | HRTIM1 三相互补 PWM,中心对齐,`Period=60000`,`fCMP≈20kHz` |
| 电流采样 | 注入组 ADC1 三相,采样电阻 2mΩ,运放增益 50(0.1V/A,8.06mA/LSB) |
| 编码器 | MT6701 磁编码器,SPI1 + DMA,14 位(0.022°/LSB) |
| 通信 | USART1 115200,命令行 + trace |
| 调试 | CMSIS-DAP(SWD),配合 OpenOCD / dap-sampler |

电机(当前装机):22 极 = **11 极对**,线间电阻约 10.2Ω,线间电感约 1.9mH。

## 软件架构

```
HRTIM (20kHz) ──ADC_TRG2──> ADC1 注入组 ──JEOS 中断──> foc_isr()
                                                        │
                        读电流 → 读编码器 → Clarke → Park → 电流PI(20kHz)
                                                        │
                                        速度PI(1kHz) ←──┤
                                        位置PD(200Hz) ←─┘
                                                        │
                                  逆Park → SVPWM → 写 HRTIM CMP
```

- **电流环 20kHz** 全部在 ADC 注入组完成中断内执行,主循环不参与控制。
- **速度环 1kHz**、**位置环 200Hz** 由 ISR 内降频计数驱动,级联关系:
  位置 PD → `speed_ref` → 速度 PI → `iq_ref` → 电流环。
- 编码器为**非阻塞 SPI+DMA** 读取:ADC ISR 末尾触发,数据在 DMA 完成中断解析。
- 主循环只做:`serial_poll()`、`foc_trace_poll()`、`openloop_trace_poll()`;开环时另调 `openloop_step()`。

## 目录结构

| 路径 | 内容 |
|---|---|
| `Core/Src/`, `Core/Inc/` | CubeMX 生成代码(仅可在 `USER CODE` 区块修改) |
| `Core/App/` | 应用层:`app_foc`(三环)、`svpwm`、`app_current_sense`、`app_openloop`、`app_serial`、`app_uart` |
| `Core/Drv/` | 驱动层:`mt6701` |
| `Core/Bsp/` | 板级支持(预留) |
| `docs/` | 代码规范、MT6701 说明、已知问题、调试记录 |

共享参数(极对数、限幅、采样系数、trace 字段)集中在 `Core/App/Inc/motor_config.h`;
各环默认增益与编码器偏移集中在 `Core/App/Inc/app_foc.h`。

## 构建与烧录

```bash
cmake --preset Debug          # 配置 → build/Debug/
cmake --build --preset Debug  # 生成 build/Debug/FOC_V0_2.elf
```

- `Debug = -O0 -g3`,`Release = -Os -g0`;工具链 `arm-none-eabi-gcc`。
- **C11 是硬要求**(`cmake/stm32cubemx/CMakeLists.txt` 会拒绝 C90/C99)。
- 烧录后 **必须手动复位**:OpenOCD 的 reset 不会真正启动固件。

## 串口命令

| 命令 | 作用 |
|---|---|
| `?` / `diag` | 状态 / 环内变量快照 |
| `sw:0xNN` | 功能位:`0x1=FOC,0x2=速度环,0x4=位置环,0x8=trace` |
| `foc on/off` | 仅开关 FOC(不改变环使能) |
| `speed:<RPM>` / `speed on/off` | 速度给定 / 速度环开关 |
| `pos <deg>` / `pos on/off` | 位置给定 / 位置环开关 |
| `iq:<A>` | 直接给定 Iq |
| `kp:` `kd:` `ki:` | 位置环参数 |
| `speed_kp:` `speed_ki:` `speed_ramp:` | 速度环参数(斜坡单位 RPM/s,0=无限) |
| `current_pid <Kp> <Ki>` | 电流环参数(不给参数则查询) |
| `pos_vmax:<deg/s>` | 位置轨迹限速 |
| `off` / `off:<deg>` | 查询/设置编码器偏移(掉电恢复宏值) |
| `trace on/off` | 200Hz `F:` trace 开关 |
| `align [volt]` + `offset` | 开环对齐 + 编码器偏移校准 |

`F:` trace 字段:`iq_ref, iq, id, theta, pos(×0.1°), speed_ref, speed_est`。

## 当前调参(2026-10-24,11 极对电机)

| 环 | 参数 | 实测 |
|---|---|---|
| 电流环 | Kp 0.44 / Ki 2385(400Hz)+ 反馈 IIR α=0.35 | 静止 iq RMS≈15mA(≈1.9 LSB,硬件本底) |
| 速度环 | Kp 0.004 / Ki 0.03,斜坡 1000 RPM/s,慢估算窗口禁用 | 0/40/100/300/500 RPM 纹波 0/25/28/13/19 RPM |
| 位置环 | Kp 1.0 / Kd 2.0,轨迹限速 360°/s | ±30~120° 超调 ≤2.2°,保持误差 ≤1.0° |
| 编码器 | 极对数 11,偏移 120° | — |
| 限幅 | Iq 1.0A | — |

## 已知问题

见 [`docs/known_issues.md`](docs/known_issues.md)。要点:

- MT6701 的 CRC 失败会静默沿用旧角度,建议增加失败计数上报。
- SPI1 配置为 `2LINES` + PA7 改为 GPIO 输入(与 `docs/mt6701.md` 的 `RXONLY` 要求不同)。
- 电流采样只用到 ADC 满量程的 1/8,信噪比受限;改善需提高运放增益或采样电阻。

## 调试工具

- **采样**:`dap-sampler` 读 RAM 变量;若出现只回低字节/乱码,先用 OpenOCD 执行一次 `init`(不复位目标)。
- **注意**:高频 SWD 采样(10kHz×多变量)会扰动目标,整定建议以 200Hz 串口 trace 为准。
