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
- Safety: Respect the active control architecture of the current branch. For the current upper-computer-driven single-gimbal path, local stop gates should be limited to explicit project rules such as NAV command freshness timeout and referee heat-limit rejection.
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
  - `CMD_NAV_DATA (0x02, LEN=27)`
    - Payload `[0..3]`: `vx` float (little-endian, m/s)
    - Payload `[4..7]`: `vy` float (little-endian, m/s)
    - Payload `[8..11]`: `vz` float (little-endian, rad/s)
    - Payload `[12..15]`: `yaw_abs` float (little-endian, rad)
    - Payload `[16..19]`: `pitch_abs` float (little-endian, rad)
    - Payload `[20]`: `nav_ctrl_flags`
      - bit0: `NAV_FLAG_CHASSIS_VALID`
      - bit1: `NAV_FLAG_GIMBAL_ABS_VALID` (legacy-compatible reserved bit; current firmware no longer requires it for gimbal execution)
      - bit2: `NAV_FLAG_FRIC_ON`
      - bit3: `NAV_FLAG_SHOOT`
    - Payload `[21..22]`: `seq` (`uint16`, little-endian)
    - Payload `[23..26]`: `timestamp_ms` (`uint32`, little-endian)
  - `CMD_GIMBAL_CALI_REQ (0x12, LEN=1)`
    - Payload `[0]`: `action`
      - `0x01`: request gimbal-only calibration
    - Firmware starts calibration only after startup delay, with recent yaw/pitch motor feedback + recent IMU solution updates, and without fresh `CMD_NAV_DATA`.
  - `CMD_HEARTBEAT (0x10, LEN=0)`: heartbeat only; refreshes `VISION_TOE` online status and does not update motion targets.
- Current firmware does **not** parse `CMD_AIM_DATA (0x01)`; any instruction or integration that assumes legacy aim packets is outdated for this repository.
- Link health for this protocol must be monitored separately in `detect_task` (`VISION_TOE`).
- `VISION_TOE` is cleared by both `CMD_NAV_DATA` and `CMD_HEARTBEAT`; however, motion commands are only refreshed by `CMD_NAV_DATA`, and the firmware clears stale motion targets if no fresh NAV frame arrives for about `100 ms`.
- For upper-computer control, prefer continuously sending full `CMD_NAV_DATA` frames instead of relying on heartbeat-only keepalive.
- Current single-gimbal runtime path uses `yaw_abs` / `pitch_abs` from `CMD_NAV_DATA` as active gimbal targets after the first valid NAV frame is received; before that it holds current position. For a stable hold after motion, the upper computer should continue sending the last desired absolute pose, while the firmware provides local hold as a stale-command fallback.
- Current shoot path is governed primarily by upper-computer `FRIC_ON` / `SHOOT` intent plus the local motor state machine; it is no longer force-stopped by gimbal behaviour mode, but referee heat-limit rejection remains an active rule when referee data is online.

## 5. Upper-Computer Feedback Rule

- UART1 upstream status feedback must use framed packets (`CMD_STATUS_REPORT = 0x81`) instead of raw bytes.
- Upstream frame (STM32 -> ROS) format is the same `SOF + CMD + LEN + payload + CRC8` format.
- `CMD_STATUS_REPORT (0x81, LEN=72)` payload currently contains:
  - `[0..1]`: robot HP (`uint16`, little-endian)
  - `[2..5]`: yaw absolute angle (`float`, little-endian)
  - `[6..9]`: pitch absolute angle (`float`, little-endian)
  - `[10]`: robot id (`uint8`)
  - `[11]`: shoot speed limit (`uint8`, currently 0)
  - `[12..13]`: last parsed NAV sequence (`uint16`, little-endian)
  - `[14..17]`: last parsed NAV timestamp from upper computer (`uint32`, little-endian)
  - `[18..19]`: NAV command age in milliseconds on STM32 (`uint16`, little-endian)
  - `[20]`: status flags (`uint8`) where bit0=`NAV_FRESH`, bit1=`VISION_ONLINE`, bit2=`REFEREE_ONLINE`, bit3=`HEAT_BLOCKED`, bit4=`GIMBAL_HOLD`, bit5=`GIMBAL_CALI`, bit6=`GIMBAL_CALI_VALID`, bit7=`GIMBAL_CALI_RUNNING`
  - `[21..24]`: `dbg_gimbal_cali_request_count` (`uint32`, little-endian)
  - `[25..28]`: `dbg_gimbal_cali_last_req_tick` (`uint32`, little-endian)
  - `[29..32]`: `dbg_gimbal_cali_now_tick` (`uint32`, little-endian)
  - `[33..36]`: `dbg_gimbal_cali_ready_tick` (`uint32`, little-endian)
  - `[37..40]`: `dbg_gimbal_cali_ready_elapsed_ms` (`uint32`, little-endian)
  - `[41..44]`: `dbg_gimbal_cali_yaw_age_ms` (`uint32`, little-endian)
  - `[45..48]`: `dbg_gimbal_cali_pitch_age_ms` (`uint32`, little-endian)
  - `[49..52]`: `dbg_gimbal_cali_imu_age_ms` (`uint32`, little-endian)
  - `[53..56]`: `dbg_gimbal_cali_dependency_max_age_ms` (`uint32`, little-endian)
  - `[57..60]`: `dbg_gimbal_cali_stable_time_ms` (`uint32`, little-endian)
  - `[61]`: `dbg_gimbal_cali_gate_state` (`uint8`)
  - `[62]`: `dbg_gimbal_cali_host_pending` (`uint8`)
  - `[63]`: `dbg_gimbal_cali_auto_pending` (`uint8`)
  - `[64]`: `dbg_gimbal_cali_pending` (`uint8`)
  - `[65]`: `dbg_gimbal_cali_running` (`uint8`)
  - `[66]`: `dbg_gimbal_cali_valid` (`uint8`)
  - `[67]`: `dbg_gimbal_cali_cmd` (`uint8`)
  - `[68]`: `dbg_gimbal_cali_block_nav_fresh` (`uint8`)
  - `[69]`: `dbg_gimbal_cali_yaw_recent` (`uint8`)
  - `[70]`: `dbg_gimbal_cali_pitch_recent` (`uint8`)
  - `[71]`: `dbg_gimbal_cali_imu_recent` (`uint8`)
- Current implementation sends this frame approximately every `50 ms` from `vision_rx_task`.
- The reported yaw/pitch values come from the active single-gimbal motor feedback (`get_yaw_motor_point()` / `get_pitch_motor_point()`), not from any dual-gimbal path.
- Calibration startup freshness must be based on real post-boot `detect_hook(...)` updates, not only `detect_init()` timestamps; if a device has not received any real update yet, its calibration dependency is not considered recent.

## 6. Debugging Practices

- Keep diagnostics lightweight in ISR contexts.
- Prefer bounded formatting APIs (`snprintf`, `vsnprintf`) over unbounded ones.
- For communication debugging, keep heartbeat/error summaries available without relying on IDE serial monitor only.
- When updating protocol-related code, keep `.github/copilot-instructions.md`, `docs/upper-computer-uart-protocol.md`, and `application/vision_rx_task.[ch]` semantically aligned.
