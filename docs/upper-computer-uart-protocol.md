# 上位机 UART 联调接口文档（当前代码实现）

本文档对应当前仓库代码实现，适用于本仓库当前版本的上位机 / 下位机 UART1 联调。

当前真实实现以以下文件为准：

- `application/vision_rx_task.h`
- `application/vision_rx_task.c`
- `application/gimbal_behaviour.c`
- `application/gimbal_task.c`
- `application/shoot.c`
- `application/detect_task.c`

> 当前正式支持路径是 **单云台 + 单拨弹**。双云台 / 双拨弹相关代码仍有历史残留，但**不属于当前受支持运行路径**。

---

## 1. 协议概览

当前协议采用 UART1 二进制帧：

- 上位机 -> STM32：
  - `CMD_NAV_DATA = 0x02`
  - `CMD_HEARTBEAT = 0x10`
- STM32 -> 上位机：
  - `CMD_STATUS_REPORT = 0x81`

当前版本新增了两类联调增强信息：

1. **命令序号 / 时间戳**
   - 上位机在 `CMD_NAV_DATA` 中携带 `seq` 与 `timestamp_ms`
   - 下位机会解析并在状态帧中回显

2. **命令新鲜度 / 最近命令信息**
   - 下位机会在状态帧中回传：
     - 最近一次 `NAV_DATA` 的 `seq`
     - 最近一次 `NAV_DATA` 的 `timestamp_ms`
     - `nav_age_ms`
     - `status_flags`

这样可以直接回答你在联调中最常见的几个问题：

- 下位机现在执行的是不是我刚发的那一帧？
- 是链路掉了，还是只是动作命令没刷新？
- 当前是不是处于本地保持位姿状态？
- 发不出去是热量限制，还是命令根本没到？

---

## 2. 帧格式

### 2.1 基本格式

| 字段 | 长度(byte) | 说明 |
| --- | ---: | --- |
| `SOF_H` | 1 | 固定 `0xA5` |
| `SOF_L` | 1 | 固定 `0x5A` |
| `CMD` | 1 | 命令字 |
| `LEN` | 1 | payload 长度 |
| `PAYLOAD` | `LEN` | 数据区 |
| `CRC8` | 1 | `CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n-1]` |

说明：

- CRC 范围只包含 `CMD + LEN + PAYLOAD`
- 不包含 `SOF_H / SOF_L`
- 当前实现是**逐字节 XOR**，不是多项式 CRC8

### 2.2 字节序

- 所有 `float` 使用 **IEEE754 单精度**
- 所有多字节整数均使用 **little-endian**
- STM32F407 当前通过 `memcpy` 直接解析字段

所以你上位机必须按 **little-endian** 打包：

- `float`
- `uint16_t`
- `uint32_t`

### 2.3 当前长度上限

- 固件允许的最大 payload：`32 byte`
- `CMD_NAV_DATA` 当前 payload：`27 byte`
- `CMD_STATUS_REPORT` 当前 payload：`21 byte`

---

## 3. 下行接口：上位机 -> STM32

## 3.1 `CMD_NAV_DATA = 0x02`

### 3.1.1 作用

统一下发：

- 底盘速度
- 云台绝对角目标
- 摩擦轮开关
- 发射请求
- 调试用命令序号
- 调试用上位机时间戳

### 3.1.2 Payload 长度

固定为 `27 byte`。

### 3.1.3 Payload 布局

| 偏移 | 类型 | 字段 | 单位 | 说明 |
| ---: | --- | --- | --- | --- |
| 0 | `float` | `vx` | m/s | 底盘前后速度 |
| 4 | `float` | `vy` | m/s | 底盘左右速度 |
| 8 | `float` | `vz` | rad/s | 底盘自转角速度 |
| 12 | `float` | `yaw_abs` | rad | 云台 yaw 绝对角目标 |
| 16 | `float` | `pitch_abs` | rad | 云台 pitch 绝对角目标 |
| 20 | `uint8_t` | `nav_ctrl_flags` | - | 控制标志位 |
| 21 | `uint16_t` | `seq` | - | 命令序号 |
| 23 | `uint32_t` | `timestamp_ms` | ms | 上位机单调递增时间戳 |

### 3.1.4 `nav_ctrl_flags` 定义

| bit | 宏名 | 值 | 含义 |
| ---: | --- | ---: | --- |
| 0 | `NAV_FLAG_CHASSIS_VALID` | `0x01` | `vx / vy / vz` 有效 |
| 1 | `NAV_FLAG_GIMBAL_ABS_VALID` | `0x02` | 保留兼容位，当前固件不再依赖它决定云台是否执行 |
| 2 | `NAV_FLAG_FRIC_ON` | `0x04` | 打开摩擦轮 |
| 3 | `NAV_FLAG_SHOOT` | `0x08` | 发射请求 |

### 3.1.5 推荐的 `seq` / `timestamp_ms` 规则

#### `seq`

- 每发送一帧新的 `CMD_NAV_DATA` 自增一次
- 推荐用 `uint16_t` 回绕即可

作用：

- 判断丢帧 / 重发
- 判断状态回传回显的是哪一帧
- 快速确认“下位机现在是不是在吃旧命令”

#### `timestamp_ms`

- 推荐使用上位机启动后的单调递增毫秒时钟
- 不要求和 STM32 的 `HAL_GetTick()` 对齐
- 它的主要用途是**上位机自己做调试对账**

作用：

- 确认最新命令是不是已经被 STM32 接收
- 对照状态回传中的回显时间戳，排查串口缓存 / 乱序 / 延迟问题

### 3.1.6 控制语义

#### 底盘控制

只有当 bit0 置位时，下位机才采用：

- `vx`
- `vy`
- `vz`

否则底盘侧将该组速度视为无效并回零输出。

#### 云台控制

当前固件会直接使用每一帧 `CMD_NAV_DATA` 中的：

- `yaw_abs`
- `pitch_abs`

作为云台绝对角目标。

说明：

- `NAV_FLAG_GIMBAL_ABS_VALID` 当前只保留兼容意义
- 固件**不再依赖**它决定云台是否执行
- 在收到第一帧有效 `NAV_DATA` 前，云台保持当前位置

#### 发射控制

当前发射路径仍保留必要执行层约束：

1. `SHOOT=1` 会隐式视为需要开摩擦轮
2. 摩擦轮要经历 ramp / ready 过程
3. 进入拨弹状态机后才会实际发射
4. 裁判系统在线且热量不足时，下位机会拒绝进入发射

---

## 3.2 `CMD_HEARTBEAT = 0x10`

### 3.2.1 作用

链路保活。

### 3.2.2 Payload 长度

固定为 `0 byte`。

### 3.2.3 当前实现语义

收到 `HEARTBEAT` 后，仅执行：

- `detect_hook(VISION_TOE)`

也就是说：

- **只刷新链路在线状态**
- **不会刷新动作命令**
- **不会刷新 `seq` / `timestamp_ms`**

### 3.2.4 一个很重要的设计点

当前固件已经把：

- **链路是否在线**
- **动作命令是否还新鲜**

拆成了两件事。

即：

- `VISION_TOE` 可以因为 `HEARTBEAT` 保持在线
- 但如果没有新的 `CMD_NAV_DATA`，动作命令仍会在约 `100 ms` 后过期并被清空

这正是为了避免“链路还在线，但车还在执行很久以前的残留命令”。

---

## 4. 上行接口：STM32 -> 上位机

## 4.1 `CMD_STATUS_REPORT = 0x81`

### 4.1.1 发送周期

当前实现约每：

$$50 \text{ ms}$$

尝试发送一帧状态。

### 4.1.2 Payload 长度

固定为 `21 byte`。

完整帧总长度为：

$$2 + 1 + 1 + 21 + 1 = 26 \text{ byte}$$

### 4.1.3 Payload 布局

| 偏移 | 类型 | 字段 | 单位 | 说明 |
| ---: | --- | --- | --- | --- |
| 0 | `uint16_t` | `hp` | - | 当前剩余血量 |
| 2 | `float` | `yaw_abs` | rad | 当前云台 yaw 实际角 |
| 6 | `float` | `pitch_abs` | rad | 当前云台 pitch 实际角 |
| 10 | `uint8_t` | `robot_id` | - | 机器人 ID |
| 11 | `uint8_t` | `shoot_speed_limit` | - | 当前实现固定为 `0` |
| 12 | `uint16_t` | `last_nav_seq` | - | 最近一次成功解析的 `NAV_DATA.seq` |
| 14 | `uint32_t` | `last_nav_timestamp_ms` | ms | 最近一次成功解析的 `NAV_DATA.timestamp_ms` |
| 18 | `uint16_t` | `nav_age_ms` | ms | 距最近一次 `NAV_DATA` 的本地经过时间 |
| 20 | `uint8_t` | `status_flags` | - | 调试状态位 |

### 4.1.4 `status_flags` 定义

| bit | 宏名 | 值 | 含义 |
| ---: | --- | ---: | --- |
| 0 | `ROS_STATUS_NAV_FRESH` | `0x01` | 最近命令仍在新鲜期内 |
| 1 | `ROS_STATUS_VISION_ONLINE` | `0x02` | `VISION_TOE` 在线 |
| 2 | `ROS_STATUS_REFEREE_ONLINE` | `0x04` | `REFEREE_TOE` 在线 |
| 3 | `ROS_STATUS_HEAT_BLOCKED` | `0x08` | 当前因热量限制不允许发射 |
| 4 | `ROS_STATUS_GIMBAL_HOLD` | `0x10` | 当前未使用新鲜 NAV 目标，云台处于本地保持位姿状态 |

### 4.1.5 字段解释

#### `last_nav_seq`

最近一次 STM32 成功解析的 `CMD_NAV_DATA.seq`。

作用：

- 和上位机当前发送序号对比，确认 STM32 是否跟上了你的控制流

#### `last_nav_timestamp_ms`

最近一次 STM32 成功解析的 `CMD_NAV_DATA.timestamp_ms`。

作用：

- 用于把上位机日志与下位机状态回传对齐

#### `nav_age_ms`

STM32 视角下，距离最近一次收到 `CMD_NAV_DATA` 已经过了多久。

说明：

- 若当前一直在稳定收命令，它应保持很小
- 若只发心跳不发 `NAV_DATA`，它会增长
- 当超过约 `100 ms` 时，动作命令会过期

#### `ROS_STATUS_GIMBAL_HOLD`

这是给联调用的一个非常实用的标志：

- 当没有新鲜 `NAV_DATA` 可用于继续驱动云台时
- 下位机会切回本地保持当前位置的策略

这个状态位可以帮助你区分：

- 是“上位机正在主动控云台”
- 还是“下位机正在本地稳住当前位姿”

---

## 5. 命令新鲜度与超时语义

当前固件定义：

$$ROS\_NAV\_CMD\_TIMEOUT\_MS = 100 \text{ ms}$$

即：

- 收到新的 `CMD_NAV_DATA`，刷新动作命令
- 超过约 `100 ms` 没有收到新的 `CMD_NAV_DATA`
  - 自动清空：
    - `vx`
    - `vy`
    - `vz`
    - `yaw_abs`
    - `pitch_abs`
    - `nav_ctrl_flags`

但是：

- 最近一次 `seq`
- 最近一次 `timestamp_ms`
- `nav_age_ms`

这些调试信息会保留下来用于状态回传。

这意味着：

- **动作会停**
- **调试证据不会丢**

这个设计非常适合上位机联调。

---

## 6. 云台保持策略：应该放在 STM32 还是上位机？

## 6.1 当前固件行为

当前代码已经具备一个本地兜底规则：

- 当有新鲜 `NAV_DATA` 时，云台执行 `yaw_abs / pitch_abs`
- 当新鲜 `NAV_DATA` 不再存在时，云台切回**保持当前位置**

也就是说，当前不会因为命令过期就把云台突然打回 `0 rad`。

## 6.2 工程上更推荐的分工

### 推荐主策略：放在上位机

对“移动之后停住，但保持当前位姿”这件事，**主策略最好放在上位机**。

也就是：

- 只要你仍处于上位机主控模式
- 每一帧都继续发送“最后希望保持的绝对角”

例如：

- 目标 yaw 已经走到 `0.32 rad`
- 目标 pitch 已经走到 `-0.08 rad`

那么在“不再继续运动”阶段，继续发送：

- `yaw_abs = 0.32`
- `pitch_abs = -0.08`

这样最好，因为：

1. 轨迹语义最清楚，谁在控一眼可知
2. 日志最好对齐，状态回传也能直接验证
3. 上位机做轨迹规划、锁定、补偿更方便

### 推荐兜底策略：保留在 STM32

STM32 端仍然保留一个兜底规则是必要的：

- 当上位机停发 `NAV_DATA`
- 或命令过期

下位机应保持当前位姿，而不是掉下去。

这条兜底策略**应该保留在 STM32**，因为它属于执行层稳定性，而不是高层规划。

### 最终建议

最稳的方案是：

- **上位机负责持续发送“最后希望保持的绝对角”**
- **STM32 负责在命令不新鲜时本地保持当前位置作为兜底**

一句话版：

> 主策略在上位机，防掉坑兜底在 STM32。

---

## 7. 推荐的上位机发送策略

## 7.1 推荐频率

建议固定频率发送 `CMD_NAV_DATA`：

- 推荐：`50 Hz ~ 200 Hz`
- 不建议低于：`20 Hz`

## 7.2 推荐每帧都发送完整状态

不要只发“变化量”。

推荐每帧都明确给出：

- `vx`
- `vy`
- `vz`
- `yaw_abs`
- `pitch_abs`
- `nav_ctrl_flags`
- `seq`
- `timestamp_ms`

## 7.3 推荐的云台保持方式

若云台已经移动到目标位姿，而你希望它稳定停住：

- 不要把 `yaw_abs / pitch_abs` 清零
- 应继续发送最后目标角

例如：

- `yaw_abs = last_target_yaw`
- `pitch_abs = last_target_pitch`

## 7.4 推荐停机方式

若你希望底盘 / 发射停止，而云台保持当前位姿：

推荐主动发送一帧：

- `vx = 0`
- `vy = 0`
- `vz = 0`
- `yaw_abs = 当前希望保持的绝对角`
- `pitch_abs = 当前希望保持的绝对角`
- `flags = 0`

随后若上位机彻底停发 `NAV_DATA`，STM32 会在约 `100 ms` 后清空动作命令，并进入本地保持位姿兜底。

---

## 8. 联调示例

## 8.1 单发示例

### 阶段 1：开摩擦轮，先不发射

- `CMD = 0x02`
- `vx = 0`
- `vy = 0`
- `vz = 0`
- `yaw_abs = 当前目标角`
- `pitch_abs = 当前目标角`
- `flags = 0x04`
- `seq = 自增`
- `timestamp_ms = 当前上位机毫秒时钟`

### 阶段 2：单发脉冲

- `flags = 0x04 | 0x08 = 0x0C`

下一周期恢复：

- `flags = 0x04`

## 8.2 全功能控制示例

若需要：

- 底盘前进 `1.0 m/s`
- 左移 `0.2 m/s`
- 自转 `0.5 rad/s`
- 云台 yaw 指向 `0.3 rad`
- 云台 pitch 指向 `-0.1 rad`
- 摩擦轮开启

则可发送：

- `vx = 1.0`
- `vy = 0.2`
- `vz = 0.5`
- `yaw_abs = 0.3`
- `pitch_abs = -0.1`
- `flags = 0x01 | 0x04 = 0x05`
- `seq = 自增`
- `timestamp_ms = 当前上位机毫秒时钟`

---

## 9. 联调时最容易踩的坑

## 9.1 `float` 按大端发送

结果：

- 底盘速度和云台角度会严重异常

结论：

- 必须使用 little-endian

## 9.2 只发一次 `NAV_DATA`，之后只发 `HEARTBEAT`

结果：

- 链路可能仍显示在线
- `status_flags` 中 `VISION_ONLINE` 仍可能为 1
- 但 `nav_age_ms` 会持续增大
- 约 `100 ms` 后 `NAV_FRESH` 会清零，动作命令被清空

结论：

- `HEARTBEAT` 不能替代持续控制帧

## 9.3 云台停止运动时把目标角清零

结果：

- 云台会朝零位走
- 可能出现你说的“突然掉下去又突然起来”

结论：

- 若你想“停住当前位姿”，应继续发送最后目标角

## 9.4 忽略热量限制

结果：

- `REFEREE_TOE` 在线且热量不足时，下位机会拒绝进入发射

结论：

- 上下位机都应维护热量策略，避免逻辑不同步

## 9.5 只看链路在线，不看命令新鲜度

结果：

- 你会误以为“既然串口在线，就应该继续运动”

结论：

- 联调时要同时看：
  - `VISION_ONLINE`
  - `NAV_FRESH`
  - `nav_age_ms`
  - `last_nav_seq`

---

## 10. 上位机最小实现建议

1. 周期性发送 `CMD_NAV_DATA`
2. 每帧带 `seq`
3. 每帧带 `timestamp_ms`
4. 所有多字节字段全部用 little-endian
5. 云台不运动时继续发送最后目标角
6. 停止时先主动发一帧零速度 + 保持姿态帧
7. 调试时同时打印：
   - 发送侧 `seq`
   - 发送侧 `timestamp_ms`
   - 回传 `last_nav_seq`
   - 回传 `last_nav_timestamp_ms`
   - 回传 `nav_age_ms`
   - 回传 `status_flags`

---
