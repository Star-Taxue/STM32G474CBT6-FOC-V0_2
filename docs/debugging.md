# 调试与调参方法

本文记录在 **WSL (Linux)** 环境下对 STM32G474 FOC 固件进行**构建、烧录、在线调参**的完整方法。

核心分工：

- **串口**：发送命令（`foc on` / `speed` / `pos` / `pid` 等）
- **SWD (DAPLink)**：高速读取内存变量（比 200Hz 的 `F:` trace 快得多）

---

## 1. 环境与工具

| 用途 | 工具 | 路径 |
|---|---|---|
| 交叉编译 | arm-none-eabi-gcc 14.3 | `E:\tool\14.3 rel1\bin` |
| 构建 | cube-cmake（STM32Cube 扩展自带） | `...\stm32cube-ide-build-cmake-1.47.0-...\cube-cmake\win32\x86_64\cube-cmake.exe` |
| ninja | STM32Cube bundle | `C:\Users\STAR\AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin` |
| 烧录 | OpenOCD 0.12.0 | `E:\tool\openocd-20250710\OpenOCD-20250710-0.12.0` |
| 变量采样 | dap-sampler v1.3.0 | `E:\software\daplink\dap-sampler-v1.3.0.exe` |

> OpenOCD、cube-cmake 等均为 Windows 程序，在 WSL 里通过 interop 直接调用（`/mnt/e/...`）。
> 前提：WSL interop 已启用（`/proc/sys/fs/binfmt_misc/WSLInterop`）。

### 1.1 把串口挂到 WSL

Windows 侧用 usbipd 把 USB 串口设备挂进 WSL：

```powershell
usbipd list
usbipd attach --wsl --busid <busid>
```

挂载后 WSL 出现 `/dev/ttyACM0`（或 `/dev/ttyUSB0`）：

```bash
ls /dev/serial/by-id/
```

本项目实测串口为 `/dev/ttyACM0`，波特率 **115200**。

> 注意：DAPLink（SWD）与串口是**两个独立 USB 设备**。挂串口不影响 Windows 侧 OpenOCD/dap-sampler 使用 DAPLink。

---

## 2. 构建

在 WSL 里用 `cmd.exe` 调用一个 bat，避免引号转义问题。

`omo_build.bat`：

```bat
@echo off
set "PATH=E:\tool\14.3 rel1\bin;C:\Users\STAR\AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin;%PATH%"
"C:\Users\STAR\.vscode\extensions\stmicroelectronics.stm32cube-ide-build-cmake-1.47.0-win32-x64\resources\cube-cmake\win32\x86_64\cube-cmake.exe" --build "E:\project\CUBEMX\STM32G474CBT6_0\FOC_V0_1\FOC_V0_2\build\Debug" --
```

执行：

```bash
cmd.exe /c 'C:\Users\STAR\AppData\Local\Temp\omo_build.bat'
```

输出末尾会打印 RAM / FLASH 占用。

> **坑**：`/mnt/e` 上的文件 mtime 有时不更新，ninja 可能误判"no work to do"。
> 若改了源码却没重编，先 `touch` 源文件再构建：
> ```bash
> touch Core/App/Src/app_foc.c Core/App/Src/app_serial.c
> ```

---

## 3. 烧录

`omo_flash.bat`：

```bat
@echo off
set OCD=E:\tool\openocd-20250710\OpenOCD-20250710-0.12.0
"%OCD%\bin\openocd.exe" -s "%OCD%\share\openocd\scripts" ^
  -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg ^
  -c "adapter speed 8000" ^
  -c "program E:/project/CUBEMX/STM32G474CBT6_0/FOC_V0_1/FOC_V0_2/build/Debug/FOC_V0_2.elf verify reset exit"
```

执行：

```bash
cmd.exe /c 'C:\Users\STAR\AppData\Local\Temp\omo_flash.bat'
```

看到 `Programming Finished` / `Verified OK` / `Resetting Target` 即成功。

### 3.1 烧录后必须手动复位

**OpenOCD 的 `reset` 不能真正启动固件**，烧录后需要**手动按开发板复位键**。
固件只在复位后运行，之后串口才有输出、SWD 变量才会更新。

### 3.2 提高烧录速度

`stm32g4x.cfg` 默认在 `reset-start` 事件里把 SWD 速度降回 2000 kHz。
若需提速，可在 openocd 里覆盖（本项目 `.vscode/launch.json` 已如此配置）：

```
-c "adapter speed 8000"
-c "stm32g4x.cpu configure -event reset-start { adapter speed 8000 }"
```

---

## 4. 串口命令

命令以 `\r\n` 结尾（**必须带换行**，否则不执行）。协议细节见 `docs/serial_protocol.md`。

常用命令：

| 命令 | 作用 |
|---|---|
| `foc on` / `foc off` | 开/关 FOC |
| `sw:0xNN` | 位掩码开关（FOC/速度环/位置环/Trace） |
| `speed <RPM>` | 速度模式并设目标 |
| `speed_pid <Kp> <Ki>` | 设速度环 PI |
| `pos <deg>` | 位置模式并设目标机械角 |
| `pos_kp <v>` / `pos_kd <v>` | 设位置环 PD |
| `pid <Kp> <Ki>` | 设电流环 PI |
| `diag` | 打印一帧诊断 |

用 Python 发命令（WSL）：

```python
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2); time.sleep(0.2)
s.reset_input_buffer()
s.write(b'speed 100\r\n'); time.sleep(0.3)
s.close()
```

---

## 5. SWD 高速变量采样（dap-sampler）

### 5.1 解析 ELF 取变量地址

固件每次重编后地址可能变化，**先 dump 取地址**：

```bash
/mnt/e/software/daplink/dap-sampler-v1.3.0.exe dump \
  "E:/project/CUBEMX/STM32G474CBT6_0/FOC_V0_1/FOC_V0_2/build/Debug/FOC_V0_2.elf" > dump.txt
grep -E 'speed_est|speed_ref|pos_current' dump.txt
```

### 5.2 单点读 / 连续采样

```bash
# 读一个 float
dap-sampler-v1.3.0.exe read 0x20000E58 --float --swd-clock 8

# 多变量连续采样到 CSV
dap-sampler-v1.3.0.exe sample \
  --addresses 0x20000E54,0x20000E58,0x20000E20 \
  --float --rate 5000 --count 5000 --swd-clock 8 > sample.csv
```

CSV 列格式：`seq,time_ms,var0,var1,var2,...`（按 `--addresses` 顺序）。

> `--swd-clock` 默认 1 MHz，采样率高时应提高到 8 MHz，否则跟不上。

### 5.3 常用变量地址（当前固件）

| 变量 | 地址 | 类型 |
|---|---|---|
| `speed_ref` | `0x20000E54` | float |
| `speed_est` | `0x20000E58` | float |
| `speed_kp` | `0x20000E5C` | float |
| `speed_ki` | `0x20000E60` | float |
| `speed_integral` | `0x20000E64` | float |
| `iq_ref` | `0x20000E20` | float |
| `pos_current` | `0x20000E48` | float |
| `pos_target_deg` | `0x20000E30` | float |
| `pos_kp` | `0x20000E34` | float |
| `pos_kd` | `0x20000E38` | float |
| `pos_mode` | `0x20000E2C` | uint8 |
| `speed_mode` | `0x20000E50` | uint8 |
| `enabled` | `0x20000E1C` | uint8 |

> 地址随代码/编译变化，**每次改代码后以 `dump` 结果为准**。

---

## 6. 调参流程

### 6.1 速度环

1. 先发 `speed <RPM>` 让电机稳定（**避免低于 60 RPM**，齿槽影响大）。
2. 用 `dap-sampler sample` 采 `speed_ref, speed_est, iq_ref`，统计纹波与标准差。
3. 用**阶跃响应**评估动态：稳态时发 `speed <新值>`，采 `speed_ref, speed_est` 看 rise/超调。
4. 先定 Kp（决定响应速度与裕度），再定 Ki（消除稳态误差）。

判读：

- `speed_est` 纹波 = 齿槽等扰动主导，PID 只能部分抑制
- 增大 Kp → 响应变快、高速纹波下降，但低速电流纹波增大（放大估算噪声）
- Kp 过大 → 振荡（`speed_est` 大幅跳变、`iq_ref` 饱和）

### 6.2 位置环

1. **先 `speed 0` 让电机停稳**（绝不能在高速下直接切位置模式）。
2. 位置环**级联到速度环**：需速度环开启（`speed` 模式），否则 `pos_out` 直连 `iq` 会饱和。
3. 设目标（`pos <deg>`），采 `pos_target_deg, pos_current` 看阶跃。
4. 注意 **0°/360° 回绕**：测试目标选在 180° 附近，避免测量被回绕污染。

判读：

- 增大 `pos_kp` → 响应变快
- `pos_kd` 提供阻尼，抑制超调
- 稳态抖动应接近编码器量化噪声量级

### 6.3 示例脚本（放在 `/tmp`，非仓库文件）

**速度稳态 + 阶跃**（核心逻辑）：

```python
import serial, time, subprocess, threading

DAP = "/mnt/e/software/daplink/dap-sampler-v1.3.0.exe"

def cmd(line):
    s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2); time.sleep(0.15)
    s.reset_input_buffer(); s.write((line+'\r\n').encode()); time.sleep(0.15); s.close()

def sample(addrs, count=5000, rate=5000):
    out = subprocess.run([DAP,'sample','--addresses',addrs,'--float',
                          '--rate',str(rate),'--count',str(count),'--swd-clock','8'],
                         capture_output=True, text=True, timeout=60).stdout
    rows = []
    for l in out.splitlines():
        try:
            v = [float(x) for x in l.split(',')]
            if len(v) >= 3: rows.append(v)
        except ValueError: pass
    return rows

# 稳态纹波
cmd('speed 100'); time.sleep(3)
rows = sample('0x20000E54,0x20000E58')   # speed_ref, speed_est
se = [r[3] for r in rows]                # 列: seq,time,var0,var1
print('ripple = %d RPM' % (max(se)-min(se)))
```

**阶跃响应**：用 `threading` 在采样同时发命令——采样线程跑 `dap-sampler`，主线程延时后发 `speed <新值>`，即可抓到动态过程（串口与 SWD 相互独立，可同时进行）。

---

## 7. 安全注意

- 切换位置模式前，**必须先让电机停稳**（`speed 0`）。
- 参数调整**逐步小步**进行，每步都观察 `speed_est` / `iq_ref` 是否稳定。
- 出现持续振荡或 `iq_ref` 饱和时，立即 `foc off`。
- 调参速度范围：**60 ~ 500 RPM**（用户指定），避免低于 60 RPM。

---

## 8. 整定结果（LD2808 700KV，12V）

| 环 | 参数 | 整定值 | 说明 |
|---|---|---|---|
| 电流环 | Kp / Ki | 0.12 / 450 | 由 R=0.2575Ω、L=68.5µH、12V 计算 |
| 速度环 | Kp / Ki | 0.15 / 0.3 | 阶跃 rise ~11ms |
| 位置环 | Kp / Kd | 4.0 / 12.0 | 30° 步进 rise ~120ms，超调 0.3°，稳态抖动 0.55° |

实测（速度环 0.15/0.3）：

| 速度 | `speed_est` 纹波 |
|---|---|
| 100 RPM | ~15 RPM (14%) |
| 500 RPM | ~14 RPM (2.8%) |
| 800 RPM | ~15 RPM (1.9%) |

> 低速纹波为齿槽转矩固有脉动，PID 无法消除（见 `docs/note.md`）。
