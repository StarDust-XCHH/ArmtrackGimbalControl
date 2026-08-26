# ArmtrackGimbalControl

[English](README.md) | [简体中文](README.zh-CN.md)

This standalone two-axis gimbal control repository was extracted from the ArmTrack project. It contains STM32F446RE firmware, Python GUI and CLI host tools, a shared ASCII serial protocol, and the UART driver used between the STM32 and two F32C motor controllers.

The repository retains only gimbal control, feedback, HOME, safe stop, and serial integration features. Vision, K230, lower-controller TI communication, shooting, laser calibration, WS2812, and SPI2 application logic are outside its scope.

## Demo Video




https://github.com/user-attachments/assets/8bc28364-a470-4b7c-a2cc-7c203f864003




## 3D Mechanical Model

The STL holder for the STM32, F32C gimbal driver, and power-related boards is archived under [`hardware/3d-models/stm32-f32c-gimbal-holder-v1/`](hardware/3d-models/stm32-f32c-gimbal-holder-v1/). That directory contains the STL, a PNG rendered from the real mesh, clockwise numbering for the four brackets as seen from the closed-face reference view, and a separate CC BY 4.0 license for the mechanical files. The largest bracket can be adapted for a power module; verify hole spacing and clearance against the physical boards before printing.

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
│   └── gimbal_cli.py           # CLI control and serial diagnostics
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
└── LICENSE                     # MIT license for firmware and Python code
```

`build/`, `cmake-build-*`, compiler outputs, Python caches, and virtual environments are generated locally and excluded by `.gitignore`; they should not be committed.

## 1. System Architecture

```text
Python GUI/CLI
       |
       | USART2, 115200 8N1
       v
STM32F446RE
       |
       | USART3, shared F32C bus, 115200 8N1
       +------ F32C yaw, address 0x01
       +------ F32C pitch, address 0x02
```

The STM32 firmware runs on FreeRTOS and uses these main tasks:

- `GimbalTask`: two-axis control, HOME state machine, feedback acquisition, speed limiting, and position holding.
- `DebugTask`: receives Python text commands through USART2 and returns acknowledgements and periodic status output.
- Default task: maintains the basic CubeMX/FreeRTOS runtime environment.

The firmware retains two mutexes: `bldcUartMutex` protects the USART3 F32C bus, and `debugUartMutex` protects USART2 output.

## 2. Implemented Features

### 2.1 Gimbal Control

- HOME to the initial pose.
- Yaw speed control.
- Single-axis yaw position control.
- Pitch position control.
- Pitch speed control.
- Combined two-axis speed/position control with `track`.
- Two-axis absolute pose control with `pose`.
- `stop`: read the current feedback and hold both axes at their current positions.
- Periodic yaw/pitch feedback acquisition and status output.
- Pitch soft limits to prevent speed mode from crossing the business-angle range.
- Safe mode transitions for yaw position control and HOME.

### 2.2 HOME Behavior

After power-up, the firmware wakes the F32C controllers and waits approximately 1500 ms. When `home` is executed:

1. Yaw speed is set to zero.
2. Yaw single-turn position mode, speed limit, and target are preloaded while the motor is disabled.
3. The yaw target is resent after approximately 100 ms.
4. The yaw target is confirmed after approximately another 50 ms.
5. Pitch timed single-turn position mode, speed limit, and target are preloaded while the motor is disabled.
6. Yaw and pitch feedback are read every 100 ms.
7. HOME succeeds only when both axis errors are no greater than 2.0 degrees for three consecutive stable samples.
8. If this condition is not met within 15 seconds, the firmware enters `HOME FAULT`.

After HOME succeeds, both axes remain in position-hold mode; the firmware does not automatically enter vision control. The normal HOME target is yaw `130.0°` and pitch `330.0°`. During HOME, the firmware reports feedback validity, current angles, and F32C UART error codes so that a motor-position problem can be distinguished from a bus with no feedback.

### 2.3 Angle Conventions

- Yaw business range: `0.0°` to `359.9°`.
- Pitch business range: `230.0°` to `405.0°`.
- Pitch motor angles remain single-turn values. Business angles from `360.0°` to `405.0°` are converted to motor angles from `0.0°` to `45.0°`; feedback is converted back to the continuous business range.
- Internal angle units are 0.1 degrees; for example, `330.0°` is represented as `3300`.

## 3. F32C Driver Implementation

[`APP/comm2motor/Src/bldc_motor.c`](APP/comm2motor/Src/bldc_motor.c) and [`APP/comm2motor/Inc/bldc_motor.h`](APP/comm2motor/Inc/bldc_motor.h) preserve the current implementation from the source project, including:

- Frame header `0x7A`, frame tail `0x7B`, and XOR BCC validation.
- Yaw address `0x01` and pitch address `0x02`.
- Speed, multi-turn position, timed single-turn position, and direct single-turn position modes.
- Enable, disable, speed, position, and feedback operations.
- Feedback-frame timeout and UART error recording.
- Approximately 2 ms spacing between bus commands.
- Power-on wake byte and 1500 ms motor-controller startup delay.

The main driver APIs used by the application are:

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

The application does not expose the low-level `BLDC_SetSingleAngle()` transition directly for yaw position commands. It uses `Gimbal_SetYawTarget()` to stop the motor, preload while disabled, re-enable it, and confirm the target according to the safe timing sequence.

## 4. Python GUI Usage

### 4.1 Install Dependencies

Python 3.10 or newer is recommended. Ensure that Tkinter is available, then run this command from the repository root:

```powershell
python -m pip install -r requirements.txt
```

`requirements.txt` currently contains only `pyserial`.

### 4.2 Start the GUI

```powershell
python tools/gimbal_gui.py
```

The GUI contains these areas:

- `Serial`: port selection, port refresh, connect/disconnect, HOME, and STOP.
- `Commands`: yaw speed, yaw position, pitch position, pitch speed, `track`, `pose`, and custom commands.
- `Status`: current angles, target angles, HOME status, and operating state.
- Log area: serial transmit/receive data, status lines, heartbeat responses, and errors.

### 4.3 Recommended Operating Procedure

1. Connect the STM32, USB-to-serial adapter, and F32C motor bus according to the wiring section.
2. Power the STM32 and motor controllers, ensuring that the mechanism can be powered off safely.
3. Start the GUI.
4. Click `Refresh` and confirm the USB-to-serial COM port in the port field.
5. Click `Connect / Disconnect` to establish the connection.
6. Confirm that `TOCK` responses and periodic status lines appear in the log.
7. Click `HOME`, wait for `GIMBAL HOME OK`, and confirm `homed:1` and `home_fault:0`.
8. Send position or speed commands only after HOME succeeds.
9. When testing is complete, click `STOP`, confirm both axes hold their current positions, and then disconnect motor power.

After connecting, the GUI sends one `tick` heartbeat every second. The heartbeat checks USART2 communication only and does not drive the motors.

### 4.4 GUI Controls

| Control | Input range | Command | Purpose |
| --- | --- | --- | --- |
| Yaw speed rpm | `-100` to `100` | `yaw <rpm>` | Yaw speed control; `0` stops yaw speed |
| Yaw position | `0.0` to `359.9` | `yawpos <angle>` | Controls yaw position only and preserves the pitch target |
| Pitch position | `230.0` to `405.0` | `pitch <angle>` | Pitch business-angle position control |
| Pitch speed rpm | `-30` to `30` | `pitchspd <rpm>` | Pitch speed control with soft limits |
| Track yaw rpm / pitch | yaw `-100..100`, pitch `230..405` | `track <yaw_rpm> <pitch>` | Combined yaw speed and pitch position control |
| Pose yaw / pitch | yaw `0..359.9`, pitch `230..405` | `pose <yaw> <pitch>` | Two-axis position control |
| HOME | None | `home` | Runs the HOME state machine |
| STOP | None | `stop` | Holds the current feedback positions of both axes |
| Send custom | ASCII text | Custom | Sends an allowed protocol command |

Angle fields accept `330`, `330.0`, and `x3300`. Both the firmware and Python protocol module convert angles to 0.1-degree units and round to the nearest 0.1 degree.

### 4.5 Status Fields

The GUI status area is populated from periodic STM32 text lines such as:

```text
yaw_cur:130.0,yaw_tgt:130.0,pitch_cur:330.0,pitch_tgt:330.0,homed:1,home_fault:0,state:3
```

- `yaw_cur`: current yaw feedback angle, or `NA` when feedback is unavailable.
- `yaw_tgt`: current yaw target angle, or `NA` in speed mode.
- `pitch_cur`: current pitch feedback angle, or `NA` when feedback is unavailable.
- `pitch_tgt`: current pitch business-angle target.
- `homed`: `1` means HOME succeeded; `0` means it has not succeeded.
- `home_fault`: `1` means HOME timed out.
- `state`: `0` means HOMING, `3` means manual control, and `4` means HOME fault.

### 4.6 GUI Troubleshooting

- The port field shows `none`: no serial port was detected. Check the USB-to-serial driver, USB connection, and COM port in Device Manager.
- The GUI connects but receives no `TOCK`: check that TX/RX are crossed, all devices share ground, and USART2 is actually routed to PA2/PA3.
- `TOCK` works but no F32C feedback is received: check USART3 wiring, motor power, common ground, 3.3 V logic compatibility, and F32C addresses.
- HOME enters fault: check F32C bus feedback first, then verify the physical yaw/pitch directions and mechanical range. Do not repeatedly send high-speed commands.
- A position command is rejected: confirm that the requested angle or speed is within the table limits.

## 5. CLI Usage

The CLI supports scripted tests, field diagnostics, and systems without a graphical desktop:

```powershell
python tools/gimbal_cli.py --port COM34
```

Send one or more commands:

```powershell
python tools/gimbal_cli.py --port COM34 --no-hex --command "home" --command "yawpos 130.0"
```

Listen for 10 seconds and send heartbeats:

```powershell
python tools/gimbal_cli.py --port COM34 --duration 10 --heartbeat --no-hex
```

The default baud rate is 115200 and can be changed with `--baud`. Received data is shown in hexadecimal by default; use `--no-hex` to disable it.

## 6. USART2 Text Protocol

All commands are ASCII text terminated by `CR/LF` (`\r\n`).

| Command | Range / format | Response or action |
| --- | --- | --- |
| `tick <seq>` | Non-negative 32-bit integer | Returns `TOCK <seq> <uptime_ms>` |
| `home` | No argument | HOME to yaw `130.0°`, pitch `330.0°` |
| `stop` | No argument | Stops current motion and holds the current position |
| `yaw <rpm>` | `-100..100` | Yaw speed control |
| `yawpos <angle>` | `0..359.9` | Single-axis yaw position control |
| `pitch <angle>` | `230..405` | Pitch position control |
| `pitchspd <rpm>` | `-30..30` | Pitch speed control |
| `track <yaw_rpm> <pitch>` | `-100..100`, `230..405` | Yaw speed plus pitch position |
| `pose <yaw> <pitch>` | `0..359.9`, `230..405` | Two-axis position control |

Removed legacy application commands such as `auto`, `vision`, `material`, `target`, `laser`, `vpid`, `left`, and `right` are not accepted.

## 7. Wiring

### 7.1 Python Host to STM32 USART2

Use a 3.3 V TTL USB-to-serial adapter. TX and RX must be crossed:

| USB-to-serial | STM32F446RE | Description |
| --- | --- | --- |
| TX | PA3 / `USART2_RX` | Host transmit to STM32 receive |
| RX | PA2 / `USART2_TX` | STM32 transmit to host receive |
| GND | GND | Common ground is required |

USART2 uses `115200 baud, 8 data bits, no parity, 1 stop bit` (115200 8N1). Do not connect RS-232 ±12 V signals directly to the STM32, and do not connect the USB-to-serial adapter to the USART3 motor bus.

### 7.2 STM32 to F32C Motor Bus on USART3

| STM32F446RE | F32C bus | Description |
| --- | --- | --- |
| PB10 / `USART3_TX` | F32C RX | STM32 transmits bus frames |
| PC5 / `USART3_RX` | F32C TX | STM32 receives feedback frames |
| GND | F32C GND | Common ground is required |

USART3 also uses `115200 8N1`. Yaw and pitch share the bus and are distinguished by F32C address: yaw is `0x01`, and pitch is `0x02`. Supply the motor and STM32 logic power according to the actual hardware design, and verify compatible logic levels.

### 7.3 Board GPIO

The project retains the original board configuration:

- PA5: onboard green LED.
- PC13: onboard push-button input.
- PA13/PA14: SWD debug interface.
- PB3: SWO.
- External clock and LSE settings remain as configured in the CubeMX project.

The LED and button remain in the board configuration, but the standalone application does not depend on an LED or button task.

## 8. Firmware Build and Flashing

### 8.1 Build Environment

Install STM32CubeCLT or an equivalent `arm-none-eabi-gcc` toolchain, CMake 3.22 or newer, and Ninja. STM32F4 HAL, CMSIS, and FreeRTOS are included in the repository.

### 8.2 Debug and Release Builds

Run these commands from the repository root:

```powershell
cmake --preset Debug
cmake --build --preset Debug

cmake --preset Release
cmake --build --preset Release
```

The primary firmware outputs are located at:

```text
build/Debug/ArmtrackGimbalControl.elf
build/Release/ArmtrackGimbalControl.elf
```

Use STM32CubeProgrammer, STM32CubeIDE, or another STM32F446RE-compatible tool to flash the ELF or converted BIN/HEX through ST-LINK. Confirm the MCU model, target power, and debug wiring before flashing.

## 9. Testing

Run the Python syntax check and protocol unit tests with:

```powershell
python -m compileall -q tools tests
python -m unittest discover -s tests -v
```

The current tests cover valid and out-of-range angles, the `xNNNN` format, rounding of multi-decimal input, speed boundaries, command formatting, `tick`/`TOCK` heartbeats, and status-line parsing.

Hardware integration testing should verify USART2 command exchange, USART3 yaw/pitch feedback, HOME success and 15-second timeout, `yawpos` preserving the pitch target, `stop` position holding, and the ranges and timing of every single-axis and dual-axis command.

## 10. Safety Notes

- Perform the first power-up and HOME test unloaded or in a low-risk setup, and verify direction and mechanical limits.
- Do not rely on position feedback before HOME succeeds.
- Keep an emergency power cutoff accessible during testing.
- Near pitch business limits `230.0°` and `405.0°`, speed mode triggers the soft limit and sets speed to zero.
- `stop` holds the current position; it does not disconnect motor power. Use hardware power cutoff first during an abnormal condition.
- Never mix the USART2 host wiring with the USART3 F32C bus.

## 11. Licensing

New application code in this repository is released under the MIT License; see [LICENSE](LICENSE). STM32 HAL, CMSIS, and FreeRTOS retain their original upstream licenses and copyright notices.
