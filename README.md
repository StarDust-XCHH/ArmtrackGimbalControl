# ArmtrackGimbalControl

[English](README.md) | [简体中文](README.zh-CN.md)

ArmtrackGimbalControl is an open-source two-axis gimbal control stack built around an STM32F446RE, FreeRTOS, Python tools, and two F32C motor controllers. It is a standalone extraction of the gimbal-control portion of the original ArmTrack project.

The repository intentionally contains only gimbal motion, feedback, HOME, safe stop, and serial integration code. Vision, K230, lower-controller TI communication, shooting, laser calibration, WS2812, and SPI2 application logic are outside this project.

## Demo Video

The video below shows the end-to-end gimbal control integration. The compressed MP4 is also stored at [`docs/demo/armtrack-gimbal-demo.mp4`](docs/demo/armtrack-gimbal-demo.mp4) for download and offline playback.

<video controls muted playsinline preload="metadata" width="100%">
  <source src="https://raw.githubusercontent.com/StarDust-XCHH/ArmtrackGimbalControl/master/docs/demo/armtrack-gimbal-demo.mp4" type="video/mp4">
  This browser does not support HTML5 video. [Download the demo video](docs/demo/armtrack-gimbal-demo.mp4).
</video>

The demo is H.264/AAC at 1920 x 1026, approximately 23 seconds long, and approximately 5.8 MB. The player uses the public repository raw URL; the repository file remains available if embedded playback is blocked by a browser or GitHub policy.

## Project Structure

```text
ArmtrackGimbalControl/
├── APP/
│   ├── arm/                    # Two-axis control, HOME state machine, angle conversion
│   ├── comm2motor/             # F32C UART frames, BCC, feedback, motor modes
│   └── debug/                  # USART2 text protocol, command parsing, status output
├── Core/
│   ├── Inc/                    # CubeMX/HAL and FreeRTOS headers
│   └── Src/                    # Clock, GPIO, USART, interrupts, task startup
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/   # STM32F4 HAL, with upstream license
│   └── CMSIS/                  # CMSIS core and RTOS interfaces
├── Middlewares/
│   └── Third_Party/FreeRTOS/   # FreeRTOS kernel, with upstream license
├── tools/
│   ├── gimbal_protocol.py      # Shared Python protocol, validation, status parsing
│   ├── gimbal_gui.py           # Tkinter graphical host controller
│   └── gimbal_cli.py           # CLI control, scripting, heartbeat, diagnostics
├── tests/
│   └── test_protocol.py        # Python protocol unit tests
├── docs/
│   └── demo/
│       └── armtrack-gimbal-demo.mp4  # Compressed integration demo
├── hardware/
│   └── 3d-models/
│       └── stm32-f32c-gimbal-holder-v1/
│           ├── *.stl            # Printable mechanical model
│           ├── *-preview.png    # Rendered STL preview
│           ├── README.md        # Bracket numbering, dimensions, print notes
│           └── LICENSE          # CC BY 4.0 for mechanical assets
├── cmake/
│   ├── gcc-arm-none-eabi.cmake # ARM GCC toolchain configuration
│   └── stm32cubemx/            # CubeMX CMake integration
├── CMakeLists.txt              # Firmware target and source organization
├── CMakePresets.json           # Debug and Release presets
├── ArmtrackGimbalControl.ioc   # STM32CubeMX peripheral/GPIO configuration
├── STM32F446XX_FLASH.ld        # STM32F446RE linker script
├── startup_stm32f446xx.s       # Cortex-M4 startup assembly
├── requirements.txt            # Python dependency list (pyserial)
└── LICENSE                     # MIT license for application code
```

`build/`, `cmake-build-*`, compiler outputs, Python caches, and virtual environments are local generated content and are excluded by `.gitignore`.

## System Architecture

```text
Python GUI / CLI
        |
        | USART2, 115200 8N1
        v
STM32F446RE
        |
        | USART3, shared F32C bus, 115200 8N1
        +------ F32C yaw, address 0x01
        +------ F32C pitch, address 0x02
```

The firmware runs on FreeRTOS. `GimbalTask` handles two-axis control, feedback, the HOME state machine, limits, and position holding. `DebugTask` receives ASCII commands on USART2 and emits acknowledgements and status lines. `bldcUartMutex` protects the USART3 F32C bus and `debugUartMutex` protects USART2 output.

## Implemented Features

- HOME to the configured initial pose.
- Yaw speed control and single-axis yaw position control.
- Pitch position control and pitch speed control.
- Dual-axis `track` and `pose` commands.
- Feedback acquisition and periodic status output.
- `stop` command that stops velocity and holds both axes at their current feedback positions.
- Pitch soft limits in speed mode.
- Safe mode transitions for yaw position commands and HOME.
- F32C UART error and feedback timeout reporting.

### HOME Behavior

After power-up, the firmware sends the F32C wake byte and waits approximately 1500 ms for the controllers to start. A `home` command then:

1. Sets yaw speed to zero.
2. Preloads yaw single-turn position mode, speed limit, and target while disabled.
3. Resends the yaw target after approximately 100 ms and confirms it after another approximately 50 ms.
4. Preloads the pitch timed single-turn position target while disabled.
5. Reads both axes every 100 ms.
6. Requires both axis errors to be at most 2.0 degrees for three consecutive stable samples.
7. Enters `HOME FAULT` after a 15-second timeout if the condition is not met.

After HOME succeeds, both axes remain in position-hold mode. The normal HOME targets are yaw `130.0°` and pitch `330.0°`.

### Angle Conventions

- Yaw business range: `0.0°` to `359.9°`.
- Pitch business range: `230.0°` to `405.0°`.
- Pitch motor angles remain single-turn values. Business angles `360.0°` to `405.0°` convert internally to motor angles `0.0°` to `45.0°`; feedback is converted back to the continuous business range.
- Firmware angles use 0.1-degree units internally.

## F32C Driver

The driver in [`APP/comm2motor`](APP/comm2motor) preserves the original implementation:

- Frame header `0x7A`, frame tail `0x7B`, and XOR BCC validation.
- Yaw address `0x01` and pitch address `0x02` on the shared bus.
- Speed, multi-turn position, timed single-turn position, and direct single-turn position modes.
- Enable, disable, speed, position, and feedback requests.
- UART error and feedback-timeout recording.
- Approximately 2 ms spacing between bus commands.
- Power-on wake byte and controller startup delay.

Application code uses safe wrapper APIs such as `BLDC_GimbalStartYawSpeed()`, `BLDC_GimbalArmYawSingleAngle()`, `BLDC_GimbalRefreshYawSingleAngle()`, `BLDC_GimbalStartPitchPosition()`, `BLDC_GimbalStartPitchSpeed()`, and `BLDC_RequestFeedbackValue()`. Yaw position commands go through `Gimbal_SetYawTarget()` instead of exposing the low-level `BLDC_SetSingleAngle()` transition directly; pitch's existing target is preserved.

## Python GUI

### Install and Start

Use Python 3.10 or newer with Tkinter available:

```powershell
python -m pip install -r requirements.txt
python tools/gimbal_gui.py
```

The GUI provides serial-port refresh and connection controls, HOME, STOP, yaw speed, yaw position, pitch position, pitch speed, `track`, `pose`, custom command input, live angles and targets, HOME state, errors, and a serial log. Once connected, it sends one `tick` heartbeat per second. The heartbeat checks USART2 communication and does not move the motors.

Recommended first-use sequence:

1. Wire the host serial adapter and F32C bus as described below.
2. Power the STM32 and motor controllers with the mechanical load in a safe state.
3. Start the GUI, click `Refresh`, select the USB serial port, and click `Connect / Disconnect`.
4. Confirm that `TOCK` and periodic status lines appear in the log.
5. Click `HOME` and wait for `GIMBAL HOME OK`, `homed:1`, and `home_fault:0`.
6. Send position or velocity commands only after HOME succeeds.
7. Click `STOP` before powering down.

| GUI control | Accepted range | Command | Function |
| --- | --- | --- | --- |
| Yaw speed | `-100..100` rpm | `yaw <rpm>` | Yaw velocity; `0` stops velocity mode |
| Yaw position | `0.0..359.9°` | `yawpos <angle>` | Yaw-only position target; pitch target is unchanged |
| Pitch position | `230.0..405.0°` | `pitch <angle>` | Pitch business-angle position target |
| Pitch speed | `-30..30` rpm | `pitchspd <rpm>` | Pitch velocity with soft limits |
| Track | yaw `-100..100`, pitch `230..405` | `track <yaw_rpm> <pitch>` | Yaw velocity plus pitch position |
| Pose | yaw `0..359.9`, pitch `230..405` | `pose <yaw> <pitch>` | Two-axis absolute pose |
| HOME | None | `home` | Run HOME state machine |
| STOP | None | `stop` | Hold current feedback positions |

## Python CLI

The CLI is useful for scripts, field diagnostics, and headless systems:

```powershell
python tools/gimbal_cli.py --port COM34
python tools/gimbal_cli.py --port COM34 --no-hex --command "home" --command "yawpos 130.0"
python tools/gimbal_cli.py --port COM34 --duration 10 --heartbeat --no-hex
```

The default baud rate is 115200. The CLI accepts repeated `--command` options, duration-based listening, heartbeat generation, hexadecimal output, and parsed status lines.

## USART2 Text Protocol

Commands are ASCII lines terminated by `CR/LF` (`\r\n`):

| Command | Range / format | Action |
| --- | --- | --- |
| `tick <seq>` | Non-negative 32-bit integer | Returns `TOCK <seq> <uptime_ms>` |
| `home` | No argument | HOME to yaw `130.0°`, pitch `330.0°` |
| `stop` | No argument | Stop velocity and hold current positions |
| `yaw <rpm>` | `-100..100` | Yaw speed control |
| `yawpos <angle>` | `0..359.9` | Single-axis yaw position control |
| `pitch <angle>` | `230..405` | Pitch position control |
| `pitchspd <rpm>` | `-30..30` | Pitch speed control |
| `track <yaw_rpm> <pitch>` | `-100..100`, `230..405` | Yaw speed plus pitch position |
| `pose <yaw> <pitch>` | `0..359.9`, `230..405` | Two-axis position control |

The removed application commands (`auto`, `vision`, `material`, `target`, `laser`, `vpid`, `left`, and `right`) are intentionally not accepted.

## Wiring

### Python Host to STM32 USART2

Use a 3.3 V TTL USB-to-serial adapter and cross TX/RX:

| USB-to-serial | STM32F446RE | Description |
| --- | --- | --- |
| TX | PA3 / `USART2_RX` | Host transmit to STM32 receive |
| RX | PA2 / `USART2_TX` | STM32 transmit to host receive |
| GND | GND | Common ground |

USART2 is `115200 8N1`. Do not connect RS-232 ±12 V signals directly to the MCU and do not attach the host adapter to USART3.

### STM32 to F32C USART3 Bus

| STM32F446RE | F32C bus | Description |
| --- | --- | --- |
| PB10 / `USART3_TX` | F32C RX | STM32 sends motor frames |
| PC5 / `USART3_RX` | F32C TX | STM32 receives feedback |
| GND | F32C GND | Common ground |

USART3 is also `115200 8N1`. Yaw and pitch share the bus and are selected by addresses `0x01` and `0x02`. Provide motor and logic power according to the hardware design, and verify signal-level compatibility.

The retained board configuration includes PA5 LED, PC13 user button, PA13/PA14 SWD, PB3 SWO, and the CubeMX clock/LSE settings. The standalone application does not depend on LED or button tasks.

## Build and Flash

Install STM32CubeCLT or an equivalent `arm-none-eabi-gcc` toolchain, CMake 3.22 or newer, and Ninja:

```powershell
cmake --preset Debug
cmake --build --preset Debug

cmake --preset Release
cmake --build --preset Release
```

Build outputs are under `build/Debug` and `build/Release`. Flash the resulting ELF, BIN, or HEX with STM32CubeProgrammer, STM32CubeIDE, or another STM32F446RE-compatible ST-LINK tool.

## Tests

```powershell
python -m compileall -q tools tests
python -m unittest discover -s tests -v
```

The protocol tests cover valid and out-of-range angles, `xNNNN` values, rounding, speed limits, command formatting, `tick`/`TOCK` heartbeats, and status-line parsing. Hardware acceptance should also verify USART2 commands, USART3 feedback, HOME success and timeout, yaw-only `yawpos` preserving the pitch target, `stop`, and all single-/dual-axis commands.

## Safety

- Perform first power-up and HOME tests unloaded or at low risk; verify direction and mechanical limits.
- Do not rely on position feedback before HOME succeeds.
- Keep an emergency power cutoff accessible.
- Pitch speed mode is clamped at business angles `230.0°` and `405.0°`.
- `stop` holds position; it is not an electrical power disconnect.
- In an abnormal condition, use hardware power cutoff first.
- Never mix the USART2 host wiring with the USART3 F32C bus.

## Licensing

New firmware and Python application code are released under the MIT License in [LICENSE](LICENSE). STM32 HAL, CMSIS, and FreeRTOS retain their upstream license notices. The mechanical model and preview image have their own [CC BY 4.0 license](hardware/3d-models/stm32-f32c-gimbal-holder-v1/LICENSE).
