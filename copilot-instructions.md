# RoboMaster Sentinel AI Coding Guidelines

This file is intentionally aligned with `.github/copilot-instructions.md` as a merged, workspace-level copy.

## 1. System Architecture and Environment
- Hardware: STM32F407 (RoboMaster C Board). Upper computer (Ubuntu 22) communicates over UART.
- Framework: STM32 HAL + FreeRTOS.
- Build: Use `MDK-ARM/standard_robot.uvprojx` in Keil MDK.
- Clean: Run `keilkilll.bat` in the repository root before pushing.

## 2. Directory Boundaries
- `Src/`, `Inc/`, `Drivers/`, `Middlewares/`: CubeMX generated. Only edit inside `/* USER CODE BEGIN ... */` and `/* USER CODE END ... */` blocks.
- `bsp/boards/`: Hardware abstraction APIs.
- `application/`: FreeRTOS tasks and high-level control logic.
- `components/`: Hardware-agnostic algorithms and reusable modules.

## 3. Coding Rules
- FreeRTOS task style: setup phase + infinite loop + explicit `osDelay(...)`.
- Naming: `snake_case` for variables/functions/files.
- Documentation: New public functions should include concise bilingual Doxygen comments.
- Safety: On link loss or invalid data, actuator commands must go to a safe state (typically zero output).

## 4. Upper-Computer Vision Protocol Rule (Critical)
- Do not reuse SBUS/DBUS payload format for upper-computer vision communication.
- UART1 vision RX must follow the frame format implemented in `application/vision_rx_task.c`:
  - Byte `[0]`: `SOF_H = 0xA5`
  - Byte `[1]`: `SOF_L = 0x5A`
  - Byte `[2]`: `CMD`
  - Byte `[3]`: `LEN` (payload length)
  - Bytes `[4..(4+LEN-1)]`: payload
  - Last byte: `CRC8` (xor of `CMD + LEN + payload`, does not include SOF)
- Supported downstream commands (ROS -> STM32):
  - `CMD_AIM_DATA (0x01, LEN=14)`
    - Payload `[0..3]`: `yaw_offset` float (little-endian)
    - Payload `[4..7]`: `pitch_offset` float (little-endian)
    - Payload `[8]`: flags (`bit0`: target_valid, `bit1`: shoot_suggest)
    - Payload `[9..10]`: `distance_mm` uint16 (little-endian)
    - Payload `[11]`: `target_id`
    - Payload `[12..13]`: reserved
  - `CMD_NAV_DATA (0x02, LEN=12)`: reserved
  - `CMD_HEARTBEAT (0x10, LEN=0)`: heartbeat
- Link health for this protocol must be monitored separately in `detect_task` (`VISION_TOE`).

## 5. Upper-Computer Feedback Rule
- UART1 upstream status feedback must use framed packets (`CMD_STATUS_REPORT = 0x81`) instead of raw bytes.
- Upstream frame (STM32 -> ROS) format is the same `SOF + CMD + LEN + payload + CRC8` format.
- `CMD_STATUS_REPORT (0x81, LEN=14)` payload currently contains:
  - `[0..1]`: robot HP (`uint16`, little-endian)
  - `[2..5]`: yaw absolute angle (`float`, little-endian)
  - `[6..9]`: pitch absolute angle (`float`, little-endian)
  - `[10]`: robot id (`uint8`)
  - `[11]`: shoot speed limit (`uint8`, currently 0)
  - `[12..13]`: reserved (`uint16`, currently 0)

## 6. Debugging Practices
- Keep diagnostics lightweight in ISR contexts.
- Prefer bounded formatting APIs (`snprintf`, `vsnprintf`) over unbounded ones.
- For communication debugging, keep heartbeat/error summaries available without relying on IDE serial monitor only.
