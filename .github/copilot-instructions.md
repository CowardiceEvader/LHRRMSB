# RoboMaster Sentinel AI Coding Guidelines

This guide informs AI agents on how to navigate, structure, and modify the LHRRMSB Sentinel project for immediate productivity. 

## 1. System Architecture & Environment
*   **Hardware**: STM32F407 (RoboMaster C Board). Upper computer (Ubuntu 22) sends vision/AIM assist data via ASRLINK (UART). Local development is on Windows 11 using Keil MDK.
*   **Framework**: STM32 HAL + FreeRTOS.
*   **Build**: Compile and flash using the Keil project (`MDK-ARM/standard_robot.uvprojx`). 
*   **Clean**: Run `keilkilll.bat` in the root directory to clear Keil intermediate files (`.o`, `.d`) before pushing to version control.

## 2. Directory Structure & Boundaries
*   `Src/`, `Inc/`, `Drivers/`, `Middlewares/`: STM32CubeMX generated files. **DO NOT modify** unless placing code specifically between `/* USER CODE BEGIN ... */` and `/* USER CODE END ... */` comments.
*   `bsp/boards/`: Board Support Package (BSP). Hardware interface layer (e.g., `bsp_can.c`, `bsp_usart.c`). **Rule**: Never call `HAL_*` functions directly in application code; always expose a `bsp_` API here.
*   `application/`: High-level FreeRTOS tasks and system logic (e.g., `chassis_task.c`, `gimbal_task.c`, `aim_assist_control.c`). 
*   `components/`: Hardware-agnostic algorithmic and math libraries (e.g., `pid.c`, `AHRS_middleware.c`).

## 3. Critical Coding Conventions
*   **FreeRTOS Tasks**: Application tasks must include a setup phase, an infinite `for(;;)` loop, and yield CPU using `osDelay()` (e.g., `osDelay(10)`).
*   **Naming Conventions**: Use `snake_case` for filenames, variables, and functions. 
*   **Documentation**: Provide Doxygen-compliant bilingual (Chinese/English) headers (`@brief`, `@param`, `@retval`) for new functions.

## 4. Safety & Debugging Patterns
*   **Fail-Safe Architecture**: The `detect_task.c` and `chassis_power_control.c` enforce hardware safety. On connection loss (e.g., UART timeout from Upper OS, CAN node offline, RC disconnect), output commands to actuators (chassis/gimbal) **must explicitly be set to safe states (e.g., 0)**. 
*   **UART/CAN Diagnostics**: When debugging lower-level communication (e.g., `application/CAN_receive.c`), rely on the `bsp_buzzer.h` APIs (like `#define CAN_FEEDBACK_BUZZER_DEBUG`) to create audible feedback heartbeat signals for the developer, independent of the Serial monitor.
*   **Mocking Inputs**: For testing chassis logic without physical commands, use conditional macros to bypass `sbus_to_rc()` inputs with simulated frames (e.g., `AIM_ASSIST_FORCE_TEST_FRAME_ONCE()` inside `test_task.c`).