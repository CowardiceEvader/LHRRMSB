# 上位机通信接口文档参考

本文档基于当前固件实现整理，适用于本仓库当前版本的上位机/下位机 UART 通信对接。

当前真实生效的协议实现来源主要是：

- `application/vision_rx_task.h`
- `application/vision_rx_task.c`
- `application/chassis_behaviour.c`
- `application/gimbal_behaviour.c`
- `application/shoot.c`
- `application/detect_task.c`

## 1. 总览

当前协议采用 **UART1 二进制帧协议**，下位机（STM32）仅接受以下两类上位机命令：

1. `CMD_NAV_DATA = 0x02`
   - 用于底盘、云台、摩擦轮、发射的统一控制
2. `CMD_HEARTBEAT = 0x10`
   - 用于保活

下位机向上位机回传：

1. `CMD_STATUS_REPORT = 0x81`
   - 用于上报机器人血量、云台当前角度、机器人 ID 等状态

> 注意：旧的 AIM/识别数据接口已经物理删除，当前协议中**不存在**旧识别控制帧。

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

### 2.2 字节序

- 所有 `float` 按 **IEEE754 单精度浮点** 发送
- 当前 STM32F407 为 **小端序**，代码直接 `memcpy` 到 `float`
- 因此上位机应按 **little-endian** 发送 `float`

### 2.3 最大长度限制

- 固件允许的最大 `payload` 长度：`32 byte`
- 当前已定义最大控制帧：`21 byte`

---

## 3. 下行接口：上位机 → 下位机

## 3.1 `CMD_NAV_DATA = 0x02`

### 3.1.1 作用

统一控制以下功能：

- 底盘平移/旋转
- 云台绝对角控制
- 摩擦轮启停
- 发射控制

### 3.1.2 Payload 长度

固定 `21 byte`

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
| 0 | `NAV_FLAG_CHASSIS_VALID` | `0x01` | `vx/vy/vz` 有效 |
| 1 | `NAV_FLAG_GIMBAL_ABS_VALID` | `0x02` | `yaw_abs/pitch_abs` 有效 |
| 2 | `NAV_FLAG_FRIC_ON` | `0x04` | 开摩擦轮 |
| 3 | `NAV_FLAG_SHOOT` | `0x08` | 发射命令 |

### 3.1.5 控制语义

#### 底盘控制

只有当 bit0 置位时，下位机才会使用：

- `vx`
- `vy`
- `vz`

否则底盘目标速度会被清零。

#### 云台控制

只有当 bit1 置位时，下位机会进入 `GIMBAL_ROS_ABSOLUTE` 模式，按：

- `yaw_abs`
- `pitch_abs`

做绝对角控制。

若 bit1 未置位，下位机会保持当前位置，不主动转动云台。

#### 摩擦轮控制

bit2 置位时，摩擦轮启动。

bit2 清零时，摩擦轮关闭，发射状态机会退回 `SHOOT_STOP`。

#### 发射控制

bit3 表示发射命令。

但请注意，发射逻辑不是“只要 bit3=1 就必发一发”这么简单，实际行为如下：

1. 必须先满足摩擦轮启动（bit2=1，或 bit3=1 时固件内部也会强制视为需要摩擦轮）
2. 摩擦轮达到 Ready 后，发射状态机才允许进入发射阶段
3. 受裁判系统热量限制约束

### 3.1.6 发射推荐用法

#### 单发推荐

推荐上位机这样做：

1. 先发送 `FRIC_ON=1`，`SHOOT=0`
2. 等待摩擦轮稳定
3. 再发送一个 **短脉冲**：`SHOOT=1`
4. 下一周期立刻恢复 `SHOOT=0`

这样最稳，等价于“单发触发”。

#### 连发推荐

若需要持续发射：

1. 保持 `FRIC_ON=1`
2. 持续保持 `SHOOT=1`

固件内会先进入单发/重复发射流程，若高电平持续时间超过约：

$$400 \text{ ms}$$

会进入连续发射模式。

#### 非常重要的注意事项

如果你在摩擦轮尚未 Ready 前就一直把 `SHOOT=1` 拉高：

- 固件会先等待摩擦轮就绪
- 但由于进入 `READY` 时 `SHOOT` 已经是高电平，**不会自动按“单发上升沿”触发一次单发**
- 持续保持足够久后，才会进入连续发射逻辑

所以对于单发，**不要从冷启动开始一直拉高 `SHOOT`**。

---

## 3.2 `CMD_HEARTBEAT = 0x10`

### 3.2.1 作用

保活帧。

### 3.2.2 Payload 长度

固定 `0 byte`

### 3.2.3 作用机制

收到后仅执行：

- `detect_hook(VISION_TOE)`

也就是：

- 刷新视觉/上位机链路在线状态
- **不会更新任何控制量**

### 3.2.4 使用建议

`HEARTBEAT` 不能替代 `NAV_DATA`。

因为当前固件逻辑下：

- `HEARTBEAT` 只保活
- 最近一次 `NAV_DATA` 的控制值会继续保留

所以如果你只发心跳、不发新的 `NAV_DATA`，下位机可能继续沿用旧控制指令。

> 建议：上位机控制周期内始终发送完整 `CMD_NAV_DATA`，`HEARTBEAT` 只作为补充，不要作为主控制帧。

---

## 4. 上行接口：下位机 → 上位机

## 4.1 `CMD_STATUS_REPORT = 0x81`

### 4.1.1 发送周期

下位机当前每隔约：

$$50 \text{ ms}$$

发送一帧状态回传。

### 4.1.2 Payload 长度

固定 `14 byte`

### 4.1.3 Payload 布局

| 偏移 | 类型 | 字段 | 单位 | 说明 |
| ---: | --- | --- | --- | --- |
| 0 | `uint16_t` | `hp` | - | 当前剩余血量 |
| 2 | `float` | `yaw_abs` | rad | 当前云台 yaw 绝对角 |
| 6 | `float` | `pitch_abs` | rad | 当前云台 pitch 绝对角 |
| 10 | `uint8_t` | `robot_id` | - | 裁判系统机器人 ID |
| 11 | `uint8_t` | `shoot_speed_limit` | - | 当前固件固定填 `0` |
| 12 | `uint16_t` | `reserved` | - | 固定 `0` |

### 4.1.4 当前实现说明

上报数据来源：

- `hp`：裁判系统 `get_robot_remain_HP()`
- `yaw_abs`：云台 yaw 电机当前绝对角
- `pitch_abs`：云台 pitch 电机当前绝对角
- `robot_id`：裁判系统 `get_robot_id()`

当前 `shoot_speed_limit` 未实际填充限制值，固定为 `0`。

---

## 5. 掉线保护与时序约束

## 5.1 视觉/上位机链路超时

在 `detect_task.c` 中，`VISION_TOE` 的配置为：

- `offlineTime = 120 ms`
- `onlineTime = 80 ms`

即：

### 超过约 120 ms 未收到 `NAV_DATA` 或 `HEARTBEAT`

会判定链路掉线。

掉线后：

- `vision_link_failsafe_clear()` 会清空：
  - `vx`
  - `vy`
  - `vz`
  - `yaw_abs`
  - `pitch_abs`
  - `nav_ctrl_flags`
- `gimbal_ros_command_active()` 返回 0
- 云台/发射输出电流会被强制清零

### 恢复在线后

需重新稳定一段时间（约 `80 ms` 在线窗口）才会被视为稳定在线。

## 5.2 裁判系统超时

在 `detect_task.c` 中，`REFEREE_TOE` 的配置为：

- `offlineTime = 100 ms`
- `onlineTime = 100 ms`

发射热量限制逻辑只有在裁判系统在线时才生效：

```text
if REFEREE online and heat + remain_value > heat_limit:
    禁止进入 SHOOT_BULLET / SHOOT_CONTINUE_BULLET
```

其中：

- `SHOOT_HEAT_REMAIN_VALUE = 80`

也就是说，固件当前采用的是：

$$heat + 80 > heat\_limit$$

则禁止继续发射。

---

## 6. 控制侧真实行为总结

## 6.1 底盘

当前底盘固定工作在：

- `CHASSIS_NO_FOLLOW_YAW`

也就是：

- 不自动跟随云台
- 直接按 `vx / vy / vz` 执行

## 6.2 云台

当前云台只认上位机绝对角控制：

- `NAV_FLAG_GIMBAL_ABS_VALID = 1` 时：
  - 执行 `yaw_abs` / `pitch_abs`
- 否则：
  - 保持当前位置

## 6.3 发射

当前发射行为受以下条件共同约束：

1. 上位机标志位
2. 摩擦轮是否 ready
3. 裁判系统热量限制
4. 链路是否在线
5. 云台是否允许发射（`gimbal_cmd_to_shoot_stop()`）

因此它不是“纯 bit3 直通电机”，而是“上位机主控 + 下位机执行安全约束”。

---

## 7. 推荐的上位机发送策略

## 7.1 推荐频率

建议上位机固定频率发送 `CMD_NAV_DATA`：

- 推荐：`50 Hz ~ 200 Hz`
- 不建议低于：`20 Hz`

只要保证链路间隔明显小于 `120 ms` 即可。

## 7.2 推荐原则

每个控制周期发送一整帧完整状态，而不是“只发变化量”。

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
- `nav_ctrl_flags = 0`

不要只停发数据等待超时。

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

如果某一时刻需要：

- 底盘前进 `1.0 m/s`
- 左移 `0.2 m/s`
- 自转 `0.5 rad/s`
- 云台 yaw 指向 `0.3 rad`
- 云台 pitch 指向 `-0.1 rad`
- 摩擦轮开启

则：

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

- 底盘速度和云台角度全乱飞

结论：

- 必须 little-endian

## 9.2 只发一次 `NAV_DATA`，后面只发 `HEARTBEAT`

结果：

- 旧控制值持续保持

结论：

- 控制周期要持续发送完整 `NAV_DATA`

## 9.3 冷启动时一直保持 `SHOOT=1` 想单发

结果：

- 不会按你预期“刚 ready 就打一发”
- 更可能等到持续高电平足够久后进入连续发射逻辑

结论：

- 单发要用脉冲

## 9.4 忽略热量限制

结果：

- 即使上位机发了开火命令，下位机也会拒绝发射

结论：

- 上位机最好同时根据裁判热量做策略层控制

---

## 10. 建议作为上位机侧的最小实现规则

1. 周期性发送 `CMD_NAV_DATA`
2. 所有 `float` 使用 little-endian
3. 每帧都带完整控制状态
4. 单发采用 `SHOOT` 脉冲
5. 连发才持续保持 `SHOOT=1`
6. 停止时主动发零控制帧
7. 上位机自身也维护热量逻辑，不要只依赖下位机拒发

---

如果后续你愿意，我还可以继续补两份配套材料：

1. **Python 上位机收发示例**（含 CRC、打包、线程收包）
2. **C++/Qt 上位机协议封装示例**（适合视觉端直接接入）
