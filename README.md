# Armtrack Gimbal Control

Standalone STM32F446RE firmware and Python tools for a two-axis F32C gimbal. This repository contains only HOME, feedback, speed control, position control and the USART2 text protocol. Vision, K230, lower-controller, shooting, laser and WS2812 application code is excluded.

## Firmware

Build with the STM32CubeCLT toolchain using `cmake --preset Debug` followed by `cmake --build --preset Debug`. Release uses the corresponding `Release` preset.

The firmware targets STM32F446RE at 180 MHz and uses FreeRTOS. USART2 is the host endpoint at 115200 8N1 (PA2 TX, PA3 RX). USART3 is the F32C motor bus at 115200 8N1 (PB10 TX, PC5 RX). The board GPIO configuration is retained.

## Text protocol

Commands are ASCII lines terminated by CR/LF: `tick <seq>`, `home`, `stop`, `yaw <rpm>`, `yawpos <0..359.9>`, `pitch <230..405>`, `pitchspd <-30..30>`, `track <yaw_rpm> <pitch>`, and `pose <yaw> <pitch>`.

Angles are business angles in degrees. `pitch` uses the continuous 230..405 degree coordinate; the motor 0..45 degree wrap is converted internally. HOME is 130.0/330.0, waits for F32C wake-up, requires both feedback axes within 2.0 degrees for three samples, and faults after 15 seconds without valid feedback.

`yawpos` follows the same safe position transition as `pose`: yaw speed is stopped, position mode and target are preloaded while disabled, the target is confirmed after a short delay, and pitch is left at its existing target.

## Python tools

Install the host dependency with `python -m pip install -r requirements.txt`.

Run `python tools/gimbal_cli.py --port COM34` for an interactive console, or use repeated `--command` arguments for scripted testing. Run `python tools/gimbal_gui.py` for the manual GUI with port refresh, HOME/STOP, yaw and pitch controls, two-axis `track`/`pose`, custom commands, status and logs.

## Wiring and safety

Connect the host serial adapter TX to PA3 (USART2_RX), adapter RX to PA2 (USART2_TX), and connect grounds. Connect the F32C yaw/pitch bus to PB10/PC5 USART3 and ground. Use 3.3 V UART levels. Test with the gimbal unloaded and keep the emergency power cutoff accessible; HOME must complete successfully before relying on position feedback.

## Tests

Run `python -m compileall tools tests` and `python -m unittest discover -s tests -v`.

The STM32 HAL, CMSIS and FreeRTOS directories retain their upstream license notices. The application code is MIT licensed.
