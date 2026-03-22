# 上位机 UART 通信协议文档

本文档基于当前仓库代码实现整理，适用于本仓库当前版本的上位机 / 下位机 UART1 对接。

当前真实生效的实现来源主要是：

- `application/vision_rx_task.h`
- `application/vision_rx_task.c`
- `application/gimbal_behaviour.c`
- `application/gimbal_task.c`
- `application/shoot.c`
- `application/detect_task.c`

> 注意：当前工程正式支持路径是 **单云台 + 单拨弹**。双云台 / 双拨弹相关代码在仓库中仍有历史残留，但**不属于当前有效协议和受支持运行路径**，本文档不对其做对接承诺。

---

## 1. 总览

当前协议采用 **UART1 二进制帧协议**。

下位机（STM32）当前只解析两类上位机下行命令：

- `CMD_NAV_DATA = 0x02`

  用于统一下发底盘、云台、摩擦轮、发射控制。

- `CMD_HEARTBEAT = 0x10`

  用于链路保活。

下位机向上位机回传：

- `CMD_STATUS_REPORT = 0x81`

  用于上报机器人血量、云台当前角度、机器人 ID 等状态。

> 当前 `application/vision_rx_task.c` **不解析**旧版 `CMD_AIM_DATA (0x01)`。任何仍假设“旧 AIM / 识别控制帧有效”的说明或上位机逻辑，均不适用于本仓库当前版本。

---

## 2. 物理链路与数据格式

### 2.1 帧格式

每一帧格式如下：

| 字段 | 长度(byte) | 说明 |
| --- | ---: | --- |
| SOF_H | 1 | 固定 `0xA5` |
| SOF_L | 1 | 固定 `0x5A` |
| CMD | 1 | 命令字 |
| LEN | 1 | Payload 长度 |
| PAYLOAD | `LEN` | 数据区 |
| CRC8 | 1 | `CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n-1]` |

说明：

- CRC 计算范围是 `CMD + LEN + PAYLOAD`
- **不包含** `SOF_H` 和 `SOF_L`
- 当前实现是按字节异或校验，不是查表 CRC8 多项式算法

### 2.2 字节序

- 所有 `float` 按 **IEEE754 单精度浮点** 发送
- 当前 STM32F407 为 **little-endian**
- 固件通过 `memcpy` 直接把 4 字节写入 `float`
- 因此上位机必须按 **little-endian** 打包浮点数

### 2.3 长度限制

- 固件允许接收的最大 `payload` 长度：`32 byte`
- 当前实际定义的最大下行控制帧：`21 byte`
- 当前上行状态帧 `payload` 长度：`14 byte`

---

## 3. 下行接口：上位机 → 下位机

## 3.1 `CMD_NAV_DATA = 0x02`

### 3.1.1 作用

统一控制以下功能：

- 底盘平移 / 旋转
- 云台绝对角控制
- 摩擦轮启停
- 发射控制

### 3.1.2 Payload 长度

固定为 `21 byte`。

### 3.1.3 Payload 布局

| 偏移 | 类型 | 字段 | 单位 | 说明 |
| ---: | --- | --- | --- | --- |
| 0 | `float` | `vx` | m/s | 底盘前后速度 |
| 4 | `float` | `vy` | m/s | 底盘左右速度 |
| 8 | `float` | `vz` | rad/s | 底盘自转角速度 |
| 12 | `float` | `yaw_abs` | rad | 云台 yaw 绝对角 |
| 16 | `float` | `pitch_abs` | rad | 云台 pitch 绝对角 |
| 20 | `uint8_t` | `nav_ctrl_flags` | - | 控制标志位 |

### 3.1.4 `nav_ctrl_flags` 定义

| bit | 宏名 | 值 | 含义 |
| ---: | --- | ---: | --- |
| 0 | `NAV_FLAG_CHASSIS_VALID` | `0x01` | `vx / vy / vz` 有效 |
| 1 | `NAV_FLAG_GIMBAL_ABS_VALID` | `0x02` | `yaw_abs / pitch_abs` 有效 |
| 2 | `NAV_FLAG_FRIC_ON` | `0x04` | 打开摩擦轮 |
| 3 | `NAV_FLAG_SHOOT` | `0x08` | 发射命令 |

### 3.1.5 控制语义

#### 底盘控制

只有当 bit0 置位时，下位机才会使用：

- `vx`
- `vy`
- `vz`

否则底盘侧会把目标速度视为无效并回到零输出。

#### 云台控制

只有当 bit1 置位时，云台行为层才会进入 `GIMBAL_ROS_ABSOLUTE`，并按：

- `yaw_abs`
- `pitch_abs`

执行绝对角控制。

若 bit1 未置位，则当前行为是：

- 不使用新的云台目标角
- 保持当前位置
- 不主动转向新的目标

#### 摩擦轮控制

bit2 置位时，摩擦轮启动。

bit2 清零时，发射状态机会退回 `SHOOT_STOP`。

#### 发射控制

bit3 表示发射请求，但发射并不是“bit3 直通电机”，实际仍受下位机状态机和安全门控约束：

1. 发射前必须处于摩擦轮工作路径
2. 当前实现中 `bit3=1` 会隐式视为需要开摩擦轮
3. 摩擦轮 ready 后才允许进入发射阶段
4. 裁判系统热量限制会拦截发射
5. 云台若处于 `GIMBAL_INIT` / `GIMBAL_CALI` / `GIMBAL_ZERO_FORCE`，发射会被强制停止

### 3.1.6 发射推荐用法

#### 单发推荐

推荐上位机按以下顺序控制：

1. 发送 `FRIC_ON=1`，`SHOOT=0`
2. 等待摩擦轮稳定
3. 发送一个短脉冲：`SHOOT=1`
4. 下一周期恢复 `SHOOT=0`

这样最稳，等价于“单发上升沿触发”。

#### 连发推荐

若需要持续发射：

1. 保持 `FRIC_ON=1`
2. 持续保持 `SHOOT=1`

固件内部在持续高电平时间超过约：

$$400 \text{ ms}$$

后，会进入连续发射模式。

该时间来自 `shoot.h` 中：

- `PRESS_LONG_TIME = 400`

而 `shoot_task` / `shoot_control_loop` 的控制周期为 `1 ms` 量级，因此这里可以近似理解为约 `400 ms`。

#### 一个非常常见的坑

如果在摩擦轮尚未 ready 前就一直保持 `SHOOT=1`：

- 固件会先等待摩擦轮进入 ready
- 但 ready 时由于 `SHOOT` 已经是高电平，不会再产生新的“上升沿”
- 更可能在持续高电平足够久后直接进入连续发射逻辑

所以对于单发，**不要从冷启动开始就一直拉高 `SHOOT`**。

---

## 3.2 `CMD_HEARTBEAT = 0x10`

### 3.2.1 作用

保活帧。

### 3.2.2 Payload 长度

固定为 `0 byte`。

### 3.2.3 作用机制

收到后当前仅执行：

- `detect_hook(VISION_TOE)`

也就是：

- 刷新上位机链路在线状态
- **不会更新任何运动目标**

### 3.2.4 使用建议

`HEARTBEAT` 不能替代 `NAV_DATA`。

因为当前实现下：

- `HEARTBEAT` 只保活
- 最近一次 `NAV_DATA` 中解析出的控制量会继续保留，直到被新 `NAV_DATA` 覆盖或链路超时触发 failsafe

所以如果你只发心跳、不发新的 `NAV_DATA`，下位机很可能继续沿用旧控制指令。

> 建议：上位机控制周期内持续发送完整 `CMD_NAV_DATA`，`HEARTBEAT` 只作为辅助保活，而不是主控制帧。

---

## 4. 上行接口：下位机 → 上位机

## 4.1 `CMD_STATUS_REPORT = 0x81`

### 4.1.1 发送周期

当前实现中，`vision_rx_task` 约每：

$$50 \text{ ms}$$

尝试发送一帧状态上报。

### 4.1.2 Payload 长度

固定为 `14 byte`。

完整帧总长度为：

$$2 + 1 + 1 + 14 + 1 = 19 \text{ byte}$$

### 4.1.3 Payload 布局

| 偏移 | 类型 | 字段 | 单位 | 说明 |
| ---: | --- | --- | --- | --- |
| 0 | `uint16_t` | `hp` | - | 当前剩余血量 |
| 2 | `float` | `yaw_abs` | rad | 当前云台 yaw 绝对角 |
| 6 | `float` | `pitch_abs` | rad | 当前云台 pitch 绝对角 |
| 10 | `uint8_t` | `robot_id` | - | 裁判系统机器人 ID |
| 11 | `uint8_t` | `shoot_speed_limit` | - | 当前实现固定填 `0` |
| 12 | `uint16_t` | `reserved` | - | 当前实现固定填 `0` |

### 4.1.4 当前实现说明

上报数据来源：

- `hp`：`get_robot_remain_HP()`
- `yaw_abs`：`get_yaw_motor_point()->absolute_angle`
- `pitch_abs`：`get_pitch_motor_point()->absolute_angle`
- `robot_id`：`get_robot_id()`

说明：

- 上报角度来自当前有效的**单云台**反馈路径
- `shoot_speed_limit` 目前未接入真实限速值，固定为 `0`
- `reserved` 当前固定为 `0`

### 4.1.5 一个实现细节

当前发送逻辑是“尽力发送”，不是排队重发：

- 当 UART1 发送状态允许时，状态帧会通过 DMA 发出
- 若该时刻 UART 仍处于忙状态，则该周期状态帧会被跳过
- 当前实现不会对这次跳过做补发排队

所以上位机应把状态上报视为**周期性 best-effort** 数据流，而不是严格无丢包遥测通道。

---

## 5. 掉线保护与时序约束

## 5.1 视觉 / 上位机链路超时

在 `detect_task.c` 中，`VISION_TOE` 配置为：

- `offlineTime = 120 ms`
- `onlineTime = 80 ms`

也就是说：

### 超过约 `120 ms` 未收到 `CMD_NAV_DATA` 或 `CMD_HEARTBEAT`

链路会被判定为掉线。

掉线后：

- `vision_link_failsafe_clear()` 会清空：
  - `vx`
  - `vy`
  - `vz`
  - `yaw_abs`
  - `pitch_abs`
  - `nav_ctrl_flags`
- `gimbal_ros_command_active()` 会返回 `0`
- `gimbal_task` 中会把：
  - yaw 电流
  - pitch 电流
  - shoot 电流
  全部强制清零

### 恢复在线后

需要重新稳定在线约：

$$80 \text{ ms}$$

才会被视为稳定在线。

## 5.2 裁判系统超时与热量限制

在 `detect_task.c` 中，`REFEREE_TOE` 配置为：

- `offlineTime = 100 ms`
- `onlineTime = 100 ms`

发射热量限制仅在裁判系统在线时参与约束。

当前代码判断逻辑为：

```text
if REFEREE online and heat + SHOOT_HEAT_REMAIN_VALUE > heat_limit:
   禁止进入 SHOOT_BULLET / SHOOT_CONTINUE_BULLET
```

其中：

- `SHOOT_HEAT_REMAIN_VALUE = 80`

即当前采用的是：

$$heat + 80 > heat\_limit$$

则拒绝继续发射。

---

## 6. 控制侧真实行为总结

## 6.1 底盘

当前 ROS / 上位机控制路径下，底盘按 `vx / vy / vz` 直接执行控制，不依赖旧识别协议。

是否采用该组速度，由 `NAV_FLAG_CHASSIS_VALID` 决定。

## 6.2 云台

当前云台正式支持的上位机控制方式是：

- `NAV_FLAG_GIMBAL_ABS_VALID = 1`
- 使用 `yaw_abs / pitch_abs`
- 进入 `GIMBAL_ROS_ABSOLUTE`
- 走绝对角控制链路

若 `NAV_FLAG_GIMBAL_ABS_VALID = 0`，则当前行为是保持当前位置，不主动切换到新的角目标。

## 6.3 发射

当前发射行为受以下条件共同约束：

1. 上位机 `FRIC_ON / SHOOT` 标志位
2. 摩擦轮是否 ready
3. 裁判系统热量限制
4. `VISION_TOE` 是否在线
5. 云台是否处于允许发射的行为模式

因此它不是“纯 bit3 直驱电机”，而是“上位机给意图、下位机做执行与安全门控”。

---

## 7. 推荐的上位机发送策略

## 7.1 推荐频率

建议上位机固定频率发送 `CMD_NAV_DATA`：

- 推荐：`50 Hz ~ 200 Hz`
- 不建议低于：`20 Hz`

原则上只要相邻有效帧间隔显著小于 `120 ms` 即可满足链路在线要求。

## 7.2 推荐原则

每个控制周期发送一整帧完整状态，而不是“只发送变化量”。

推荐每帧都明确给出：

- `vx`
- `vy`
- `vz`
- `yaw_abs`
- `pitch_abs`
- `nav_ctrl_flags`

这样最不容易出现旧指令残留。

## 7.3 推荐单发流程

1. 发送 `FRIC_ON=1, SHOOT=0`
2. 等待摩擦轮稳定
3. 某一周期发送 `SHOOT=1`
4. 下一周期恢复 `SHOOT=0`

## 7.4 推荐停机流程

若需要明确停机，建议主动发送一帧：

- `vx = 0`
- `vy = 0`
- `vz = 0`
- `yaw_abs = 当前希望保持的角度` 或 `0`
- `pitch_abs = 当前希望保持的角度` 或 `0`
- `nav_ctrl_flags = 0`

不要只停发数据等待超时，因为那会把“停机”变成“依赖掉线保护”。

---

## 8. 示例

## 8.1 单发示例

### 阶段 1：开摩擦轮，先不发射

- `CMD = 0x02`
- `vx = 0`
- `vy = 0`
- `vz = 0`
- `yaw_abs = 当前目标角`
- `pitch_abs = 当前目标角`
- `flags = 0x02 | 0x04`

含义：

- 云台角度有效
- 摩擦轮打开
- 不发射

### 阶段 2：单发脉冲

- `flags = 0x02 | 0x04 | 0x08`

下一周期：

- `flags = 0x02 | 0x04`

## 8.2 全功能控制示例

若某一时刻需要：

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
- `flags = 0x01 | 0x02 | 0x04 = 0x07`

---

## 9. 对接时最容易踩的坑

## 9.1 把 `float` 按大端发了

结果：

- 底盘速度和云台角度会严重异常

结论：

- 必须使用 little-endian

## 9.2 只发一次 `NAV_DATA`，后面只发 `HEARTBEAT`

结果：

- 旧控制值仍会继续生效，直到掉线保护触发

结论：

- 控制周期要持续发送完整 `NAV_DATA`

## 9.3 冷启动时一直保持 `SHOOT=1` 想单发

结果：

- 不会按“刚 ready 就打一发”的直觉工作
- 更可能在持续高电平足够久后进入连续发射

结论：

- 单发要用脉冲

## 9.4 忽略热量限制

结果：

- 即使上位机发了开火命令，下位机也可能拒绝发射

结论：

- 上位机最好也维护自己的热量策略层逻辑

## 9.5 把双云台残留代码当成正式接口

结果：

- 对接方向会跑偏
- 文档和实际执行链会对不上

结论：

- 当前请只按**单云台主链路**对接

---

## 10. 上位机最小实现建议

1. 周期性发送 `CMD_NAV_DATA`
2. 所有 `float` 使用 little-endian
3. 每帧都带完整控制状态
4. 单发采用 `SHOOT` 脉冲
5. 连发才持续保持 `SHOOT=1`
6. 停止时主动发零控制帧
7. 上位机也应维护热量 / 状态策略，不要完全依赖下位机拒发

---

如果后续需要，我还可以继续补两份配套材料：

1. `Python` 上位机收发示例（含打包、CRC、线程收包）
2. `C++ / Qt` 协议封装示例（适合视觉端直接接入）
