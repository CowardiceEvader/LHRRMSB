# RoboMaster Sentinel — Copilot / AI 代理使用说明（合并版）

本文件为对仓库内现有说明的扩展与实操指南，面向在本针对特定的 RoboMaster 哨兵项目运行自动化编码代理（如 GitHub Copilot、Cursor）或进行代码改动的工程师。

## 核心概览（快速扫描）
- **项目类型**：STM32F407 嵌入式 C 语言开发，基于 STM32CubeMX 生成的 HAL 库 + FreeRTOS 实时操作系统。
- **目标硬件**：使用的是**大疆 RoboMaster 开发板 C 板**。
- **开发与构建工具链**：Keil MDK（工程文件主入口：`MDK-ARM/standard_robot.uvprojx`）。
- **主要目录结构**：
  - `Src/`, `Inc/`, `Drivers/`, `Middlewares/`：CubeMX 自动生成的启动代码、HAL 库及外设初始化文件。
  - `bsp/boards/`：板级驱动包（BSP），统一封装并对外暴露硬件接口（前缀为 `bsp_*`，如 `bsp_can.c`），用于隔离底层硬件逻辑。
  - `application/`：核心应用层。包含业务逻辑实现与各个 FreeRTOS 任务（底盘、云台、发射、遥控、裁判系统等）。

## 开发环境与物理拓扑（极其重要）
- **开发机（本电脑）**：**Windows 11** 系统。主要职责：运行 Keil 编译和烧录 C 板，并使用串口调试助手接收下位机信息、进行人为调试。
- **上位机（运算侧）**：**Ubuntu 22** 系统电脑。主要职责：运行视觉、导航或AI策略，向下位机传输自瞄与导航控制数据。
- **传输媒介**：上下位机间的数据通信依赖 **ASRLINK 自动下载器**（作为 USB 转串口模块透传通信）。

## 关键规则（必须遵守）
- **保护生成代码**：**绝对不要直接改动** CubeMX 管理的自动生成代码。如需手动修补，必须且只能将手写代码放置在 `/* USER CODE BEGIN ... */` 与 `/* USER CODE END ... */` 区间内，否则重新生成时会被覆盖。
- **命名与注释规范**：
  - 变量与函数命名采用 `snake_case`。
  - 强制使用中英双语注释，函数头部须遵循 Doxygen 标签规范（`@brief`、`@param`、`@retval`）。
- **任务架构约定**：所有的 FreeRTOS 任务文件必须包含一次性初始化模块和绝对死循环 `for(;;)`，并且在循环体内必须使用 `osDelay()` 释放执行权并做周期等待。
- **BSP 优先原则**：与硬件外设相关的操作必须经过 `bsp/boards/` 中的封装层接口实现。**严禁**在 `application/` 应用层中直接调用 `HAL_*` 库函数（若无现成接口，应先在 BSP 层补充实现）。

## 常见子系统与定位示例
- **底盘 / CAN 电机控制**：`application/CAN_receive.c`，`bsp/boards/bsp_can.c`，`application/chassis_power_control.c`。
- **云台控制算法**：`application/gimbal_task.c`，`application/gimbal_behaviour.c`。
- **遥控器通信（DJI DBUS 接收）**：`application/remote_control.c`（依赖 UART + DMA 持续接收）。
- **裁判系统解析**：`application/referee.c`，`application/referee_usart_task.c`。

## 构建 / 清理 / 烧录
- **构建工程**：通过 Keil MDK 打开 `MDK-ARM/standard_robot.uvprojx` 点击编译并下载直接烧录到 C 板。
- **产物清理**：由于 Keil 生成的临时文件体积巨大，提交代码或打包前，请在根目录执行 `keilkilll.bat` 脚本清除中间产物（*.o, *.d 等）。

## 安全策略（控制系统的底线）
- 出现任何关键性软硬件错误（如：RC 遥控器信号丢失、CAN 节点/电机离线、陀螺仪/IMU 传感器异常等），设备必须立即进入安全模式。
- **动作**：将所有执行机构（特别是电机）的设定输出置为安全值（通常为 `0`）。
- **依赖文件**：重点关注并维护 `application/detect_task.c`（离线及异常检测中心）与 `application/chassis_power_control.c`。

## 对自动化代理（AI Agent）的实操模板
1. **代码修改流**：在 `application/` 目录下做业务逻辑改动时，若涉及到底层硬件变动，应同时去 `bsp/boards/` 完成必要的驱动封装。
2. **安全挂钩**：新增逻辑后，请务必增加或兼容已有的安全检查（RC 时效状态、模块离线检测、PID 输出限幅）。
3. **隔离解耦**：在 PR 描述或代码调整中，应当说明如何在无物理硬件连接的环境下，通过宏（Macro）或运行时配置安全地禁用电机端物理输出，以方便脱机调试。
4. **注释契约**：严格保留并补充 Doxygen 注释，说明清楚新接口的输入 / 输出契约关系。

## 注意事项
- 本仓库以 Keil 专有配置为主，暂未提供 Makefile/CMake 原生命令行构建脚本文件；如后续需要接入 CI/CD，请提前与维护者沟通构建方案迁移。
- 考虑到双系统联调环境（Windows 开发 + Ubuntu 运行算法），凡涉及 `aim_assist_control` (自瞄/上位机辅助控制) 的串口解析逻辑，修改时必须谨慎考虑通信两端存在的系统差异（如串口流控、粘包断帧、大小端等问题）。
- 在修改电机控制或 PWM 自动输出相关的代码时，必须优先保证能通过纯软件开关或宏指令**关闭硬件级输出**，以防在带电调试时发生设备暴走等危险事故。