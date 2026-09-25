# AGENTS.md — FOC_V0_2 (STM32G474CBT6)

FOC 电机控制固件（STM32CubeMX 生成 + CMake/Ninja + `arm-none-eabi-gcc`）。
当前已不是骨架：包含可运行的电流采样、SVPWM、开环驱动、三环 FOC、MT6701 编码器与串口命令行。

## 最高优先级约束

- **本对话所有回答必须使用中文。**
- **不要主动运行 cmake / 构建 / 烧录 / 调试命令** — 用户通过 VS Code STM32CubeIDE 插件构建，agent 只负责写代码。
- **`Core/` 生成代码只能在 `/* USER CODE BEGIN */` … `/* USER CODE END */` 区块内修改**，其余区域 CubeMX 重新生成会覆盖。
- **不要编辑 `Drivers/`（HAL/CMSIS）与 `cmake/stm32cubemx/CMakeLists.txt`**（后者会被 CubeMX 重生成）。
- 新增用户源文件/头文件必须手动加入**根目录 `CMakeLists.txt`** 的 `target_sources()` / `target_include_directories()`。

## 构建（仅供理解，勿主动执行）

```
cmake --preset Debug          # 配置 → build/Debug/
cmake --build --preset Debug  # 构建 → build/Debug/FOC_V0_2.elf
```
- `Debug` = `-O0 -g3`；`Release` = `-Os -g0`。默认 Debug。
- 生成 `build/Debug/compile_commands.json`，`.clangd` 指向它（clangd 索引依赖先配置过一次）。
- VS Code 用 `cube-cmake` / `starm-clangd` 及 `CUBE_BUNDLE_PATH`；**没有测试、没有 CI**，唯一验证途径是编译 + 上板。

## 目录分层

| 路径 | 用途 |
|---|---|
| `Core/Src/`, `Core/Inc/` | CubeMX 生成：`main.c` 负责初始化与主循环接线 |
| `Core/App/` | 应用层：`app_foc`、`svpwm`、`app_current_sense`、`app_openloop`、`app_serial`、`app_uart`；`Inc/motor_config.h` 为共享参数**单一事实来源** |
| `Core/Drv/` | 驱动层：`mt6701` 磁编码器 |
| `Core/Bsp/` | 板级支持（**当前为空**，尚未使用） |
| `docs/` | `一般代码规范.md`、`mt6701.md`、MT6701 数据手册 PDF |

## 运行时架构（读文件名看不出来，重点）

- **20kHz 电流环在中断里跑**：HRTIM 触发 ADC1 注入组 → `JEOC` 中断调用 `foc_isr()`（Clarke→Park→PI→逆Park→SVPWM→写 PWM）。主循环**不**执行控制算法。
- 主循环只做三件事：`serial_poll()`、`foc_trace_poll()`、`openloop_trace_poll()`；开环模式下另调 `openloop_step()`。
- 三环级联：位置 PD 200Hz（`/100`）→ 速度 PI 1kHz（`/20`）→ 电流 PI 20kHz。
- 编码器为**非阻塞 SPI+DMA**读取：开销在 DMA 完成中断，主循环/ISR 调 `mt6701_read_angle()` 取缓存。
- 串口命令行解析在 `Core/App/Src/app_serial.c`：UART RX 中断 → 环形缓冲 → 行解析 → 命令分发。新增/改名命令时同步更新该文件的 `?` 帮助文本。
- 上电流程：先启动 HRTIM 计数与 ADC 做**电流零点校准**（电机须静止），再使能 PWM 输出。

## 时钟与外设（与旧文档不同，以此为准）

- 时钟：HSI 16 MHz → PLL `M=DIV4, N=75, P=DIV2` → **SYSCLK 150 MHz**（**不是 170 MHz**）；`PWR_REGULATOR_VOLTAGE_SCALE1`，Flash 4 等待周期。
- PWM/电流环 20 kHz：`Core/Src/hrtim.c` 中 `Period = 60000`（Up-Down）。
  ⚠️ `motor_config.h` 的 `HRTIM_PERIOD=30000` 与注释不一致；**实际周期以 `hrtim.c` 的 60000 为准**，`CMP_CENTER=30000`。
- 外设：`HRTIM1`（PWM + `ADC_TRG2` 触发）、`ADC1`（注入组 3 相电流，`JEOC`）、`DMA1`（SPI1 RX + ADC）、`SPI1`（MT6701）、`USART1`（调试 + 命令）、`GPIO`。
- 初始化顺序（`main.c`）：`HAL_Init` → `SystemClock_Config` → `MX_GPIO_Init` → `MX_DMA_Init` → `MX_USART1_UART_Init` → `MX_SPI1_Init` → `MX_HRTIM1_Init` → `MX_ADC1_Init` → 用户初始化。

## 工具链

- CPU：`-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`；`-Wall -fdata-sections -ffunction-sections`。
- 链接脚本 `STM32G474XX_FLASH.ld`（128K Flash @ `0x08000000`，128K RAM @ `0x20000000`），`--gc-sections`，输出 `.map` 与内存占用。
- 宏：`USE_HAL_DRIVER`、`STM32G474xx`（Debug 另有 `DEBUG`）。
- **C11 是硬要求**（`cmake/stm32cubemx/CMakeLists.txt` 会对 C90/C99 直接 `message(ERROR)`）。`docs/一般代码规范.md` 写的 "C99" 已失效，**以 C11 为准**。

## 代码规范

遵循 `docs/一般代码规范.md`（详细版），最易踩的几条：

- 4 空格缩进，**禁用 Tab**；关键字与 `(` 间空格、函数名与 `(` 无空格。
- 注释只用 `/* */`（**禁止 `//`**）；函数/结构/枚举用 Doxygen（`\brief`、`\param[in]`）。
- 变量/函数小写，宏全大写；指针星号贴类型（`uint8_t* p`）。
- **不使用 `stdbool.h`**（用 `uint8_t` + 1/0）；用 `stdint.h` 定宽类型；优先前自增 `++i`；指针与 `NULL` 比较。
- 函数实现返回类型单独一行；头文件需 `#ifndef` 保护 + `extern "C"`；包含顺序：对应 `.h` → 标准库 → 项目头。

## 硬件 / 驱动陷阱

- **MT6701（`PA6` MISO 与 `PA7` MOSI 硬件短接）**：SPI1 必须配置为 `SPI_DIRECTION_2LINES_RXONLY`，并把 `PA7` 重配为 `GPIO_MODE_INPUT`，否则 MOSI 推挽输出会破坏读取（表现为全 0）。采样用 **SPI Mode 1（CPHA=1，下降沿）**；上电后至少等 100ms 再读。覆盖配置须放在 `USER CODE BEGIN SPI1_Init 2` 并重新 `HAL_SPI_Init()`。详见 `docs/mt6701.md`。
- **`printf` 不支持 `%f`**：需要 `target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)`（约 +10KB Flash）。**当前根 `CMakeLists.txt` 未添加**，因此代码中避免 `%f`、统一用整数强转打印；若确需浮点输出，先加该链接选项。

## Git 注意事项

- 工作区存在 **CRLF/LF 行尾漂移**：`git status` 会显示约 1150 个文件"被修改"（几乎全部 `Drivers/`、`Core/Src/`），实际内容未变。**不要 `git add -A` / `git commit -a`**，只暂存你自己编辑的文件；提交前用 `git diff -- <path>` 确认真实改动。
- `Core/App/` 下可能有**真实未提交改动**（如 `app_foc.*`、`app_serial.c`），修改前先确认工作区状态，勿覆盖用户进行中的工作。
- `.gitignore`：忽略 `build/`、`mx.scratch`；`.settings/` 被跟踪。

## 参考文档

- `docs/一般代码规范.md` — C 代码风格（含反例）。
- `docs/mt6701.md` — 编码器协议/时序/SPI 配置/坑位。
- `docs/MT6701_Rev.1.8_中文版.pdf` — 数据手册。
