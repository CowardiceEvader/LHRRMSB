# RoboMaster Sentinel AI Coding Guidelines

This guide defines how AI agents should modify the LHRRMSB Sentinel firmware safely and consistently.

## 1. System Architecture and Environment

- Hardware: STM32F407 (RoboMaster C Board). Upper computer (Ubuntu 22) communicates over UART.
- Framework: STM32 HAL + FreeRTOS.
- Build: Use `MDK-ARM/standard_robot.uvprojx` in Keil MDK.
- Clean: Run `keilkilll.bat` in the repository root before pushing.
- Current active control architecture is **single gimbal + single trigger**.
- Dual-gimbal / dual-trigger code paths are historical or experimental leftovers and are **not part of the supported runtime path** for this repository.

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
- Unless the user explicitly requests hardware re-architecture, do **not** extend, repair, or enable dual-gimbal behaviour as part of normal feature work.

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
  - `CMD_NAV_DATA (0x02, LEN=21)`
    - Payload `[0..3]`: `vx` float (little-endian, m/s)
    - Payload `[4..7]`: `vy` float (little-endian, m/s)
    - Payload `[8..11]`: `vz` float (little-endian, rad/s)
    - Payload `[12..15]`: `yaw_abs` float (little-endian, rad)
    - Payload `[16..19]`: `pitch_abs` float (little-endian, rad)
    - Payload `[20]`: `nav_ctrl_flags`
      - bit0: `NAV_FLAG_CHASSIS_VALID`
      - bit1: `NAV_FLAG_GIMBAL_ABS_VALID`
      - bit2: `NAV_FLAG_FRIC_ON`
      - bit3: `NAV_FLAG_SHOOT`
  - `CMD_HEARTBEAT (0x10, LEN=0)`: heartbeat only; refreshes `VISION_TOE` online status and does not update motion targets.
- Current firmware does **not** parse `CMD_AIM_DATA (0x01)`; any instruction or integration that assumes legacy aim packets is outdated for this repository.
- Link health for this protocol must be monitored separately in `detect_task` (`VISION_TOE`).
- `VISION_TOE` is cleared by both `CMD_NAV_DATA` and `CMD_HEARTBEAT`; on timeout, `vision_link_failsafe_clear()` zeros chassis velocity, gimbal targets, and control flags.
- For upper-computer control, prefer continuously sending full `CMD_NAV_DATA` frames instead of relying on heartbeat-only keepalive.

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
- Current implementation sends this frame approximately every `50 ms` from `vision_rx_task`.
- The reported yaw/pitch values come from the active single-gimbal motor feedback (`get_yaw_motor_point()` / `get_pitch_motor_point()`), not from any dual-gimbal path.

## 6. Debugging Practices

- Keep diagnostics lightweight in ISR contexts.
- Prefer bounded formatting APIs (`snprintf`, `vsnprintf`) over unbounded ones.
- For communication debugging, keep heartbeat/error summaries available without relying on IDE serial monitor only.
- When updating protocol-related code, keep `.github/copilot-instructions.md`, `docs/upper-computer-uart-protocol.md`, and `application/vision_rx_task.[ch]` semantically aligned.
