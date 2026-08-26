# ArmtrackGimbalControl

[English](README.md) | [简体中文](README.zh-CN.md)

这是一个从 ArmTrack 工程中独立出来的双轴云台控制仓库，包含 STM32F446RE 固件、Python 上位机 GUI/CLI、共享 ASCII 串口协议，以及 STM32 与两个 F32C 电机控制器之间的 UART 驱动。

本仓库只保留云台控制、反馈、HOME、安全停止和串口联调功能。视觉、K230、下位机 TI 通信、射击、激光校准、WS2812 和 SPI2 业务均不在本仓库范围内。

## 3D 机械模型

用于安装 STM32、F32C 云台驱动板和电源相关板卡的 STL 支架已归档在 [`hardware/3d-models/stm32-f32c-gimbal-holder-v1/`](hardware/3d-models/stm32-f32c-gimbal-holder-v1/)。该目录包含 STL、由真实网格渲染的 PNG 预览、封口面视角下的四个挂架顺时针编号说明，以及独立的 CC BY 4.0 机械文件许可。模型的最大挂架可按实际板卡尺寸改作电源模块挂架；打印前请用实物核对孔距和净空。

## 演示视频

下面的视频展示了本项目云台控制端到端联调效果。视频文件同时保存在 [`docs/demo/armtrack-gimbal-demo.mp4`](docs/demo/armtrack-gimbal-demo.mp4)，便于下载和离线查看。

<video controls muted playsinline preload="metadata" width="100%">
  <source src="https://raw.githubusercontent.com/StarDust-XCHH/ArmtrackGimbalControl/master/docs/demo/armtrack-gimbal-demo.mp4" type="video/mp4">
  当前浏览器不支持 HTML5 视频播放，请使用<a href="docs/demo/armtrack-gimbal-demo.mp4">视频文件链接</a>下载查看。
</video>

视频为 H.264/AAC、1920 x 1026、约 23 秒，压缩后约 5.8 MB。README 播放器使用公开仓库的 raw 地址；仓库 push 后即可在 GitHub 项目页加载，若浏览器或 GitHub 安全策略阻止内嵌播放，仍可点击上面的仓库文件链接。

## 项目目录

```text
ArmtrackGimbalControl/
├── APP/
│   ├── arm/                    # 双轴云台控制、HOME 状态机和角度转换
│   ├── comm2motor/             # F32C UART 帧、BCC 校验、反馈和电机模式驱动
│   └── debug/                  # USART2 文本协议、命令解析和状态输出
├── Core/
│   ├── Inc/                    # CubeMX/HAL 和 FreeRTOS 头文件
│   └── Src/                    # 时钟、GPIO、USART、中断和任务初始化
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/   # STM32F4 HAL（保留上游许可证）
│   └── CMSIS/                  # CMSIS 核心和 RTOS 接口
├── Middlewares/
│   └── Third_Party/FreeRTOS/   # FreeRTOS 内核（保留上游许可证）
├── tools/
│   ├── gimbal_protocol.py      # Python 共享协议、范围检查和状态解析
│   ├── gimbal_gui.py           # Tkinter 图形上位机
│   └── gimbal_cli.py           # 命令行联调和监听工具
├── tests/
│   └── test_protocol.py        # Python 协议单元测试
├── docs/
│   └── demo/
│       └── armtrack-gimbal-demo.mp4  # 压缩后的联调演示视频
├── hardware/
│   └── 3d-models/
│       └── stm32-f32c-gimbal-holder-v1/
│           ├── *.stl            # 可打印机械模型
│           ├── *-preview.png    # STL 真实网格预览
│           ├── README.md        # 挂架编号、尺寸和打印说明
│           └── LICENSE          # 机械文件 CC BY 4.0
├── cmake/
│   ├── gcc-arm-none-eabi.cmake # ARM GCC 工具链配置
│   └── stm32cubemx/            # CubeMX 生成代码的 CMake 集成
├── CMakeLists.txt              # 固件目标和源文件组织
├── CMakePresets.json           # Debug/Release 配置
├── ArmtrackGimbalControl.ioc   # STM32CubeMX 外设/GPIO 配置
├── STM32F446XX_FLASH.ld        # STM32F446RE 链接脚本
├── startup_stm32f446xx.s       # Cortex-M4 启动文件
├── requirements.txt            # Python 依赖（pyserial）
└── LICENSE                     # 固件和 Python 应用 MIT License
```

`build/`、`cmake-build-*`、编译产物、Python 缓存和虚拟环境属于本地生成内容，已由 `.gitignore` 排除，不需要提交到仓库。

## 1. 系统组成

```text
Python GUI/CLI
       |
       | USART2, 115200 8N1
       v
STM32F446RE
       |
       | USART3, F32C 总线, 115200 8N1
       +------ F32C yaw，地址 0x01
       +------ F32C pitch，地址 0x02
```

STM32 固件使用 FreeRTOS，主要任务如下：

- `GimbalTask`：双轴控制、HOME 状态机、反馈读取、速度限幅和位置保持。
- `DebugTask`：USART2 接收 Python 文本命令、返回应答和周期性输出状态。
- 默认任务：保持 CubeMX/FreeRTOS 基础运行环境。

固件保留两个互斥锁：`bldcUartMutex` 保护 USART3 F32C 总线，`debugUartMutex` 保护 USART2 输出。

## 2. 已实现功能

### 2.1 云台控制

- HOME 回初始姿态。
- yaw 速度控制。
- yaw 单轴位置控制。
- pitch 位置控制。
- pitch 速度控制。
- 双轴速度/位置组合控制：`track`。
- 双轴绝对姿态控制：`pose`。
- `stop`：读取当前反馈并在当前位置保持。
- yaw/pitch 周期性反馈读取和状态输出。
- pitch 软限位，防止速度模式越过业务角度范围。
- yaw 位置控制和 HOME 的安全模式切换。

### 2.2 HOME 行为

上电后 F32C 唤醒并等待约 1500 ms。执行 `home` 时：

1. yaw 速度设为 0。
2. yaw 在禁用状态下预加载单圈位置模式、速度限制和目标角度。
3. 等待约 100 ms 后重新发送 yaw 目标。
4. 等待约 50 ms 后确认 yaw 目标。
5. pitch 在禁用状态下预加载定时单圈位置模式、速度限制和目标角度。
6. 每 100 ms 读取 yaw/pitch 反馈。
7. 两轴误差均不超过 2.0°，并连续稳定 3 次，才判定 HOME 成功。
8. 15 s 内没有满足条件则进入 `HOME FAULT`。

HOME 成功后继续保持位置模式，不会自动进入视觉控制。当前普通 HOME 目标为 yaw `130.0°`、pitch `330.0°`。HOME 过程中会输出反馈有效性、当前角度和 F32C UART 错误码，便于区分电机未到位和总线没有反馈。

### 2.3 角度定义

- yaw 业务范围：`0.0°` 至 `359.9°`。
- pitch 业务范围：`230.0°` 至 `405.0°`。
- pitch 电机角度仍为单圈角度。业务角度 `360.0°` 至 `405.0°` 会转换为电机 `0.0°` 至 `45.0°`，反馈读取时再转换回连续业务角度。
- 角度内部单位为 0.1°，例如 `330.0°` 表示 `3300`。

## 3. F32C 驱动实现

`APP/comm2motor/Src/bldc_motor.c` 和 `APP/comm2motor/Inc/bldc_motor.h` 沿用源工程当前版本，未改变底层协议实现，包括：

- 帧头 `0x7A`、帧尾 `0x7B` 和 BCC 异或校验。
- yaw 地址 `0x01`，pitch 地址 `0x02`。
- 速度、多圈位置、单圈定时位置和单圈直接位置模式。
- 使能、禁用、速度、位置和反馈读取。
- 反馈帧超时和 UART 错误记录。
- 每条总线指令之间约 2 ms 的间隔。
- 上电 wake byte 和 1500 ms 电机启动等待。

上层使用的主要驱动接口包括：

```c
BLDC_GimbalPowerOnWake();
BLDC_GimbalStartYawSpeed();
BLDC_GimbalArmYawSingleAngle();
BLDC_GimbalRefreshYawSingleAngle();
BLDC_GimbalStartPitchPosition();
BLDC_GimbalArmPitchPosition();
BLDC_GimbalStartPitchSpeed();
BLDC_GimbalRefreshPitchSpeed();
BLDC_RequestFeedbackValue();
```

应用层不直接使用底层 `BLDC_SetSingleAngle()` 实现 yaw 位置命令，而是通过 `Gimbal_SetYawTarget()` 执行停止、禁用预加载、重新使能和目标确认的安全时序。

## 4. Python GUI 使用方法

### 4.1 安装依赖

建议使用 Python 3.10 或更高版本，并确保系统安装 Tkinter。在仓库根目录执行：

```powershell
python -m pip install -r requirements.txt
```

`requirements.txt` 当前只包含 `pyserial`。

### 4.2 启动 GUI

```powershell
python tools/gimbal_gui.py
```

GUI 启动后包含以下区域：

- `Serial`：串口号、刷新串口、连接/断开、HOME、STOP。
- `Commands`：yaw 速度、yaw 位置、pitch 位置、pitch 速度、track、pose 和自定义命令。
- `Status`：当前角度、目标角度、HOME 状态和运行状态。
- 日志区：显示串口收发、状态行、心跳应答和错误信息。

### 4.3 推荐操作流程

1. 按照“接线方式”连接 STM32、USB 转串口模块和 F32C 电机总线。
2. 给 STM32 和电机控制器供电，确认电机处于可安全断电状态。
3. 启动 GUI。
4. 点击 `Refresh`，在串口号输入框中确认 USB 转串口对应的 COM 口。
5. 点击 `Connect / Disconnect` 建立连接。
6. 观察日志区是否收到 `TOCK` 和周期性状态行。
7. 点击 `HOME`，等待日志显示 `GIMBAL HOME OK`，并确认 `homed:1`、`home_fault:0`。
8. HOME 成功后再执行位置或速度控制。
9. 测试完成后点击 `STOP`，确认两轴保持当前位置，再断开电机电源。

连接成功后 GUI 会每秒自动发送一次 `tick` 心跳。心跳只用于检查 USART2 通信，不会驱动电机。

### 4.4 GUI 控件说明

| 控件 | 输入范围 | 发送命令 | 用途 |
| --- | --- | --- | --- |
| Yaw speed rpm | `-100` 至 `100` | `yaw <rpm>` | yaw 速度控制，`0` 表示停止 yaw 速度 |
| Yaw position | `0.0` 至 `359.9` | `yawpos <angle>` | 只控制 yaw 位置，pitch 目标保持不变 |
| Pitch position | `230.0` 至 `405.0` | `pitch <angle>` | pitch 业务角度位置控制 |
| Pitch speed rpm | `-30` 至 `30` | `pitchspd <rpm>` | pitch 速度控制，带软限位 |
| Track yaw rpm / pitch | yaw `-100..100`，pitch `230..405` | `track <yaw_rpm> <pitch>` | yaw 速度与 pitch 位置组合控制 |
| Pose yaw / pitch | yaw `0..359.9`，pitch `230..405` | `pose <yaw> <pitch>` | 双轴姿态位置控制 |
| HOME | 无 | `home` | 执行 HOME 状态机 |
| STOP | 无 | `stop` | 保持当前两轴反馈位置 |
| Send custom | ASCII 文本 | 自定义 | 发送协议中允许的命令 |

角度输入支持 `330`、`330.0` 和 `x3300`。固件和 Python 协议模块都会将角度转换为 0.1° 单位，并按 0.1° 四舍五入。

### 4.5 状态字段说明

GUI 状态区来自 STM32 周期性输出的文本行，例如：

```text
yaw_cur:130.0,yaw_tgt:130.0,pitch_cur:330.0,pitch_tgt:330.0,homed:1,home_fault:0,state:3
```

- `yaw_cur`：yaw 当前反馈角度，无反馈时为 `NA`。
- `yaw_tgt`：yaw 当前目标角度，速度模式下为 `NA`。
- `pitch_cur`：pitch 当前反馈角度，无反馈时为 `NA`。
- `pitch_tgt`：pitch 当前业务目标角度。
- `homed`：`1` 表示 HOME 成功，`0` 表示未成功。
- `home_fault`：`1` 表示 HOME 超时故障。
- `state`：`0` 正在 HOME，`3` 手动控制，`4` HOME 故障。

### 4.6 GUI 常见问题

- 显示 `none`：系统没有发现串口，检查 USB 转串口驱动、USB 连接和设备管理器中的 COM 号。
- 能连接但没有 `TOCK`：检查 TX/RX 是否交叉连接、是否共地，以及 USART2 是否确实为 PA2/PA3。
- 有 `TOCK` 但没有 F32C 反馈：检查 USART3 接线、电机供电、总线共地、3.3 V 电平和 F32C 地址。
- HOME 进入故障：先检查 F32C 总线反馈，再确认 yaw/pitch 实际运动方向和机械范围；不要重复发送高速控制命令。
- 位置命令被拒绝：检查角度和速度是否在表格范围内。

## 5. CLI 使用方法

CLI 适合脚本测试、现场排障和没有图形桌面的环境：

```powershell
python tools/gimbal_cli.py --port COM34
```

发送一次或多次命令：

```powershell
python tools/gimbal_cli.py --port COM34 --no-hex --command "home" --command "yawpos 130.0"
```

监听 10 秒并发送心跳：

```powershell
python tools/gimbal_cli.py --port COM34 --duration 10 --heartbeat --no-hex
```

CLI 默认波特率为 115200，也可以通过 `--baud` 修改。默认显示接收十六进制数据；使用 `--no-hex` 可以关闭。

## 6. USART2 文本协议

所有命令均为 ASCII 文本，并以 `CR/LF`（`\r\n`）结束。

| 命令 | 范围/格式 | 应答或作用 |
| --- | --- | --- |
| `tick <seq>` | 非负 32 位整数 | 返回 `TOCK <seq> <uptime_ms>` |
| `home` | 无参数 | 请求 HOME 到 yaw `130.0°`、pitch `330.0°` |
| `stop` | 无参数 | 停止当前速度并保持当前位置 |
| `yaw <rpm>` | `-100..100` | yaw 速度控制 |
| `yawpos <angle>` | `0..359.9` | yaw 单轴位置控制 |
| `pitch <angle>` | `230..405` | pitch 位置控制 |
| `pitchspd <rpm>` | `-30..30` | pitch 速度控制 |
| `track <yaw_rpm> <pitch>` | `-100..100`、`230..405` | yaw 速度 + pitch 位置 |
| `pose <yaw> <pitch>` | `0..359.9`、`230..405` | 双轴位置控制 |

当前不会接受 `auto`、`vision`、`material`、`target`、`laser`、`vpid`、`left`、`right` 等旧业务命令。

## 7. 接线方式

### 7.1 Python 上位机到 STM32 USART2

使用 3.3 V TTL 电平 USB 转串口模块，必须交叉连接收发线：

| USB 转串口 | STM32F446RE | 说明 |
| --- | --- | --- |
| TX | PA3 / `USART2_RX` | 上位机发送，STM32 接收 |
| RX | PA2 / `USART2_TX` | STM32 发送，上位机接收 |
| GND | GND | 必须共地 |

USART2 参数：`115200 baud, 8 data bits, no parity, 1 stop bit`（115200 8N1）。不要将 RS-232 ±12 V 信号直接接到 STM32，也不要把 USB 转串口模块接到 USART3 电机总线。

### 7.2 STM32 到 F32C 电机总线 USART3

| STM32F446RE | F32C 总线 | 说明 |
| --- | --- | --- |
| PB10 / `USART3_TX` | F32C RX | STM32 发送总线帧 |
| PC5 / `USART3_RX` | F32C TX | STM32 接收反馈帧 |
| GND | F32C GND | 必须共地 |

USART3 同样使用 `115200 8N1`。yaw 和 pitch 共享该总线，通过 F32C 地址区分：yaw 为 `0x01`，pitch 为 `0x02`。电机电源和 STM32 逻辑电源按实际硬件设计供电，并确保逻辑信号电平兼容。

### 7.3 板载 GPIO

当前工程沿用原板配置：

- PA5：板载绿色 LED。
- PC13：板载按键输入。
- PA13/PA14：SWD 调试。
- PB3：SWO。
- 外部时钟和 LSE 配置保持 CubeMX 工程设置。

LED 和按键保留为板级配置，但当前独立业务不依赖 LED 任务或按键任务。

## 8. 固件构建与烧录

### 8.1 构建环境

需要安装 STM32CubeCLT 或等效的 `arm-none-eabi-gcc`、CMake 3.22 或更高版本以及 Ninja。STM32F4 HAL、CMSIS 和 FreeRTOS 已保留在仓库中。

### 8.2 Debug/Release 构建

在仓库根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug

cmake --preset Release
cmake --build --preset Release
```

主要固件产物位于：

```text
build/Debug/ArmtrackGimbalControl.elf
build/Release/ArmtrackGimbalControl.elf
```

可使用 STM32CubeProgrammer、STM32CubeIDE 或其他支持 STM32F446RE 的烧录工具通过 ST-LINK 烧录 ELF/转换后的 BIN/HEX 文件。烧录前确认芯片型号、供电和调试接口连接正确。

## 9. 测试

Python 代码检查和协议单元测试：

```powershell
python -m compileall -q tools tests
python -m unittest discover -s tests -v
```

当前测试覆盖合法/越界角度、`xNNNN` 格式、多位小数四舍五入、速度边界、命令格式化、`tick`/`TOCK` 心跳和状态行解析。

硬件联调至少应验证：USART2 收发命令、USART3 读取 yaw/pitch 反馈、HOME 成功和 15 s 超时、`yawpos` 不改变 pitch 目标、`stop` 保持当前位置，以及各单轴/双轴命令的范围和时序。

## 10. 安全注意事项

- 首次上电和首次 HOME 必须卸载或低风险测试，确保机械限位和方向正确。
- HOME 未成功前，不要依赖位置反馈进行自动化运动。
- 测试时保持紧急断电开关可用。
- 速度模式接近 pitch `230.0°` 或 `405.0°` 时，固件会触发软限位并将速度置零。
- `stop` 是保持当前位置命令，不等同于切断电机电源；发生异常时应优先使用硬件断电。
- 不要将 USART2 上位机线和 USART3 F32C 总线混接。

## 11. 许可证

本仓库新增应用代码采用 MIT License，见 [LICENSE](LICENSE)。STM32 HAL、CMSIS 和 FreeRTOS 目录保留其原有上游许可证和版权声明。
