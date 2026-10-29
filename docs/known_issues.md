# 已知问题(2026-10-23 电机换装后审查)

> 记录**尚未修改**的问题。按优先级排列。

## 中优先级

### 1. `off:<deg>` 设置的偏移会被 `foc_start()` 覆盖
- 位置:`Core/App/Src/app_foc.c` → `foc_start()`:`encoder_offset_rad = FOC_ENCODER_OFFSET_DEG * DEG_TO_RAD;`
- 影响:`off:` 调好偏移后,任何 `sw:` 切换(重入 `foc_start`)都会打回宏值 120,容易误判。
- 建议:`foc_start()` 不再从宏加载(仅 `foc_init()` 加载),或增加 override 标志。

### 2. 位置环积分限幅量纲错位
- 位置:`Core/App/Src/app_foc.c` 位置环 `pos_integral` 用 `FOC_IQ_MAX` 限幅。
- 影响:速度环级联时 `pos_out` 被当作 RPM 使用,`pos_ki > 0` 时积分被限制在 ±1 RPM,几乎失效(`pos_ki` 默认 0,当前无影响)。
- 建议:按 `speed_mode` 选择限幅值(`FOC_SPEED_MAX_RPM` 或 `FOC_IQ_MAX`)。

### 3. SPI1 方向与 `docs/mt6701.md` 不一致
- 位置:`Core/Src/spi.c` → `hspi1.Init.Direction = SPI_DIRECTION_2LINES;`(文档要求 `SPI_DIRECTION_2LINES_RXONLY`)。
- 现状:USER CODE 里只把 PA7 改成 GPIO 输入,未重新 `HAL_SPI_Init()`;短接问题被规避,但配置与文档不符。
- 影响:偶发编码器 CRC 失败的风险(与"偶发抖动"相关)。
- 建议:按文档在 `USER CODE BEGIN SPI1_Init 2` 中改方向并重新 `HAL_SPI_Init()`。

## 低优先级

### 4. 极对数重复定义
- `Core/App/Inc/motor_config.h: MOTOR_POLE_PAIRS`(开环用)与 `Core/App/Inc/app_foc.h: FOC_POLE_PAIRS`(FOC 用)需手动保持一致。
- 建议:`FOC_POLE_PAIRS` 直接引用 `MOTOR_POLE_PAIRS`。

### 5. `FOC_SPEED_MAX_RPM = 10000` 不合理
- 11 极对下 10k RPM → 1.83 kHz 电频率;IQ_MAX=1A 时该限幅无实际作用。
- 建议:降到约 2000。

### 6. 20 kHz ISR 内的忙等与关中断
- `mt6701_trigger_read()` 内 `delay_us(2)`,约占 50 µs 周期的 4%。
- `mt6701_read_angle()` 在 ISR 内 `__disable_irq()/__enable_irq()`。
- 建议:去掉忙等,ISR 内不必关中断。

### 7. 编码器 CRC 失败不可见
- `Core/Drv/Src/mt6701.c`:CRC 失败时 `g_cached = g_last_valid`(静默沿用旧角度),`g_crc_fail_count` 未在任何命令输出;FOC ISR 也不检查 `crc_valid`。
- 建议:`diag` 命令输出 CRC 失败计数/数据新鲜度,便于定位偶发问题。

---

## 附:当前有效参数(2026-10-24)

| 项 | 值 | 依据 |
|---|---|---|
| 极对数 | 11(22 极) | 开环电频率 vs 编码器机械角实测 |
| 编码器偏移 | 120° | 实测 110~125 均可;210 时零转矩锁死 |
| 电流环 Kp / Ki | **0.22 / 1193** | 相值 R=5.12 Ω、L=0.948 mH,带宽 400 Hz;**按 24V 母线归一化(0.45·Vdc=10.8)** |
| 电流反馈 IIR α | **0.35** | 极点约 1.4 kHz,对 400 Hz 环仅 16° 滞后 |
| 速度环 Kp / Ki | **0.004 / 0.02** | 24V:100/300/1000/1900 RPM std 7/2/7~9/12~13 RPM;1700 RPM 有轻微共振(pp 48~71) |
| 慢速估算窗口 | **禁用**(`FOC_SPEED_SLOW_WIN_RPM = 0`) | 10 ms 慢窗口在 <60 RPM 必然极限环 |
| IQ 限幅 / 速度限幅 | **1.0 A / 2000 RPM** | 24V 实测可到 1950 RPM;1A 对应铜损约 7.7W |
| 位置环 Kp / Kd | **1.0 / 2.0** | ±30°/±60°/±120° 阶跃:超调 ≤2.2°、保持误差 ≤1.0°、保持峰峰 ≤1.2°(1.5/3 更快但 -120° 不可重复) |

### 整定过程中的关键结论

1. **低速抖动是确定性的,不是偶发**:`|speed| < 60 RPM` 时 10 ms 慢窗口使速度环相位越过 -180°,必然振荡(实测 40 RPM 纹波 ~470 RPM、0 RPM 纹波 ~600 RPM)。禁用慢窗口后 0/40 RPM 全部稳定。
2. **高频 SWD 采样会扰动目标**:10 kHz × 4 变量时读数完全失真(转速推算 400 RPM 而实际 100 RPM);降到 10 Hz 即正常。整定应以 200 Hz 串口 trace 为准。
3. **电流环噪声已到硬件本底**:静止 iq RMS 从 88.1 mA(3.31/17880 无滤波)→ 21.8 mA(0.88/4770、α=0.45)→ **14.9 mA**(0.44/2385、α=0.35),约 1.85 LSB。采样链 2 mΩ×50 → 8.06 mA/LSB,1 A 仅用满量程 1/8,继续降噪需提高运放增益或增大采样电阻。
4. **滤波与带宽必须配套**:α=0.3(极点 1.1 kHz)配 1.5 kHz 电流环会失稳;α=0.35 配 400 Hz 才安全。

