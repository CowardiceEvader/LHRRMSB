# 云台标定流程文档（无遥控器 / 上位机触发版）

本文档介绍如何在**没有遥控器**的情况下，通过上位机（ROS / Ubuntu）发送 UART1 指令来完成云台标定。

适用对象：

- yaw 电机：`GM6020`（当前固件映射到 `CAN ID 6 / 0x20A`）
- pitch 电机：`GM6020`（当前固件映射到 `CAN ID 5 / 0x209`）
- 总线：`CAN2`

> 根据你提供的 GM6020 说明书，GM6020 支持 **CAN / PWM 双模式**，并能自动识别控制模式；而本仓库当前云台实现明确使用 **CAN 控制**，不是 PWM 控制。

---

## 1. 标定原理概述

### 1.1 标定做了什么

云台标定会自动让云台依次执行以下四步：

1. **Pitch 正向极限**：pitch 电机以固定电流正向旋转，直到撞到机械限位（陀螺仪检测到角速度接近零），记录此时编码器和角度。
2. **Pitch 反向极限**：pitch 电机反向旋转到另一侧机械限位，记录编码器和角度。
3. **Yaw 正向极限**：yaw 电机正向旋转到限位，记录。
4. **Yaw 反向极限**：yaw 电机反向旋转到限位，记录。

四步全部完成后，固件会计算：

- **yaw / pitch 编码器中值（offset_ecd）**：即云台"正中间"位置对应的编码器值
- **yaw / pitch 最大/最小相对角度（max/min_relative_angle）**：可用旋转范围

结果写入 **Flash**，下次上电自动加载，无需重新标定。

### 1.2 什么时候需要标定

- 首次上电（Flash 中没有有效校准数据）
- 更换云台电机或机械结构后
- 观察到云台上电后一直处于零力（`GIMBAL_ZERO_FORCE`）状态，不响应命令
- 状态回传中 `ROS_STATUS_GIMBAL_CALI_VALID`（bit6）始终为 0

### 1.3 先说一个很容易误判的点：GM6020 断电并不会“自锁”

如果你的判断依据是：

- 云台在**没通电**时不能停在任意位置
- 手一松就会因为重力或惯性回落 / 转动

那么这**不能直接说明电机坏了**。

从第一性原理看：

- GM6020 是直驱无刷云台电机，本质上不是蜗轮蜗杆或带机械抱闸的执行器
- 没有通电电流，就没有电磁转矩去对抗重力和扰动
- 所以“断电不能稳住姿态”通常是**正常现象**，尤其是 pitch 轴承受重力矩时更明显

真正有诊断意义的是：

- **上电后有没有收到 CAN 控制与反馈**
- **上电后 pitch 是否能产生明显抗扰力矩**
- **编码器反馈 / 状态位是否在线**

### 1.4 自动标定

固件默认启用了上电自动标定（`AUTO_GIMBAL_CALI_ENABLE = 1`）：

- 当 Flash 中没有有效标定数据时
- 等待 yaw/pitch 电机反馈与 IMU 解算输出都持续刷新
- 且当前没有新鲜 `CMD_NAV_DATA`（即上位机没在发控制命令）
- 稳定等待约 100ms 后自动开始

当前版本额外做了一个区分：

- **上电自动标定**：仍使用更严格的门控（默认 `20ms recent + 100ms stable window`）
- **上位机主动重标**：使用更宽松的门控（默认 `100ms recent + 20ms stable window`）

这样做的原因是：

- 自动标定发生在刚上电阶段，需要更保守
- 主机主动重标通常发生在系统已经稳定运行之后，不应因为短暂抖动长期卡在 `pending`

> **如果自动标定已经成功完成并写入 Flash，后续上电将跳过标定直接进入正常控制。**
> **如果你发现上电后没有蜂鸣器响，不代表“标定执行权已经交给上位机”。** 当前架构仍然是**下位机本地执行标定扫位**。
> 只有在以下条件同时满足时才会响并开始：Flash 中当前没有有效云台标定数据；yaw / pitch / IMU 最近持续有**真实更新后的**新鲜数据；当前没有新鲜 `CMD_NAV_DATA`。
> 只要其中任一条件不满足，就会表现成“上电不叫”。

---

## 2. 前提条件

在触发标定前，建议先做一个 **GM6020 硬件健康检查**，否则会出现“流程没错，但电机根本没进入控制”的情况。

### 2.1 硬件健康检查（强烈建议先做）

| 检查项 | 正常现象 | 异常时说明 |
| --- | --- | --- |
| 断电手拨 pitch 轴 | 可被外力拨动，不会自锁 | **正常**，不能据此判断电机坏了 |
| 上电后电机灯亮 | 说明供电到达 | **只能说明上电，不代表 CAN 正常** |
| yaw 能动、pitch 不动 | 说明至少 CAN2 主链路和 yaw 那一路有通信 | 更像是 **pitch 单独那一路** 有问题，而不是整条 CAN2 都反了 |
| 触发标定时 pitch 完全不动 | 标定流程不会自然把“坏链路”修好 | 优先怀疑 `pitch ID`、`pitch 分支 CAN 线`、`pitch 电机/驱动器故障`、机械卡死 |

### 2.2 当前固件的 CAN 映射（必须确认）

当前代码固定写死为：

- `pitch = GM6020 ID5 = 0x209`
- `yaw = GM6020 ID6 = 0x20A`
- 控制报文发送到 `0x2FF`

也就是说：

1. 如果你的**上下轴电机拨码 ID 不是 5 / 6**，固件就会控不到
2. 如果 **yaw 在动但 pitch 不动**，那么：
    - **整条 CAN2 的 CANH/CANL 不太可能全反**，否则 yaw 也不会正常
    - 更可能是 **pitch 这一支路接触不良 / CANH-CANL 接反 / ID 不对 / 终端电阻设置不对 / 电机驱动损坏**

### 2.2.1 关于你现在这根“红黑两线 CAN”的判断方法

如果你的云台 CAN 是一条两芯线，先接一个电机，再从这个电机继续串到另一个电机，那么这种 **头接一个、尾接一个的串接方式本身是正常的**，这就是典型的 CAN 总线串联拓扑。

要点如下：

- CAN 是一条**差分总线**，关键不是“谁在头、谁在尾”，而是 **CANH / CANL 是否一一对应**
- 也就是说：控制板 `CANH` 要接电机 `CANH`，控制板 `CANL` 要接电机 `CANL`
- 线的颜色 **红 / 黑 并没有行业统一定义**，不能只靠颜色判断哪根一定是 `H`、哪根一定是 `L`

所以：

- **串接方式本身一般没有问题**
- 真正有问题的是：某一段把 `CANH` 和 `CANL` 交叉接反了，或者某个接头虚接了

结合你现在的现象：

- `pitch` 电机上电亮 **5 下**，说明它的 **ID 很大概率就是 5**
- 如果 `yaw` 那路还能工作，那么 **整条主干 CAN2 大概率是通的**
- 这时更该怀疑的是：
    1. `pitch` 电机这一路的 `H/L` 在局部被接反
    2. `pitch` 接头接触不良
    3. `pitch` 电机虽然 ID 对了，但反馈/驱动异常

### 2.2.2 终端电阻也会影响稳定性

GM6020 说明书提到可以配置 CAN 终端电阻。对这种一条总线串两个设备的场景，通常原则是：

- **总线物理两端** 各保留一个终端电阻
- 中间节点不要重复再加很多终端

如果终端配置混乱，常见现象是：

- 有时候能通信，有时候不稳定
- 某一个节点比另一个节点更容易掉线

不过从你的描述看，如果 `yaw` 基本正常、只有 `pitch` 异常，那么仍然更像是 **pitch 局部接线 / 接头 / 电机本体** 问题，而不是整条 CAN 总线拓扑本身有根本错误。

### 2.3 标定前的最小前提

在触发标定前，请确认以下条件：

| 条件 | 说明 |
| --- | --- |
| CAN 总线正常 | yaw / pitch 电机在 CAN2 上正常通信 |
| 电机已上线 | 上电后 `YAW_GIMBAL_MOTOR_TOE` 和 `PITCH_GIMBAL_MOTOR_TOE` 已在线 |
| IMU 已上线 | 板载 IMU (`RM_IMU_TOE`) 已持续有新鲜姿态解算输出 |
| 无新鲜 NAV 命令 | 上位机**停止发送** `CMD_NAV_DATA`（否则标定被推迟） |
| 云台可自由旋转 | 机械上 yaw / pitch 都能安全地转到极限位置再弹回来 |

> 重点：当前固件在 `auto_start_gimbal_calibrate()` 里要求 **yaw / pitch / IMU 最近都确实在持续刷新数据** 才会开始自动或主机触发标定。也就是说，只要 pitch 那路 CAN/ID 有问题，或者 IMU 解算链根本没在刷新，标定就不会真正开始。

---

## 3. 上位机发送标定指令

### 3.0 先回答一个核心问题：没有遥控器，是否只要上位机发 yaw 数据就能标定？

答案是：**不完全是。**

更准确地说：

- **是的**，没有遥控器也可以完整标定
- **但不是**靠上位机持续发送 `yaw_abs / pitch_abs` 目标来“手动带着标定”

当前固件的真实机制是：

1. 上位机只负责发送一条 **“开始标定”请求**（`CMD_GIMBAL_CALI_REQ`）
2. 下位机收到请求后，进入 `GIMBAL_CALI` 模式
3. 下位机内部用**固定电流 raw 控制**自动完成四步扫限位：
    - pitch max
    - pitch min
    - yaw max
    - yaw min
4. 扫限位完成后，下位机自己计算 offset / min / max，并写入 Flash

所以从第一性原理看：

- **遥控器版** 本质是“人为触发标定模式”
- **上位机版** 本质是“串口命令触发标定模式”

两者在**触发意图**上等价，但在**标定执行过程**上，真正干活的是下位机内部状态机，而不是上位机持续发 yaw 数据。

### 3.0.1 当前版本补充结论：不是“全交给上位机”

如果你是因为“上电后不再自动蜂鸣”才怀疑现在改成了上位机接管，那么这里明确说明：

- **没有**改成“上位机下发整套扫位轨迹”
- **没有**改成“上位机持续发 yaw/pitch 来完成标定”
- 现在仍然是：**上位机只触发，下位机本地扫位、本地判限位、本地写 Flash**

因此，上位机联调侧需要关注的是：

1. 什么时候停发 `CMD_NAV_DATA`
2. 什么时候发一次 `CMD_GIMBAL_CALI_REQ`
3. 怎么看 `gimbal_cali / gimbal_cali_running / gimbal_cali_valid`

而不是去生成一串标定动作轨迹。

### 3.1 帧格式

使用 `CMD_GIMBAL_CALI_REQ = 0x12`，payload 长度 1 byte。

完整帧（6 bytes）：

```text
[0xA5] [0x5A] [0x12] [0x01] [0x01] [CRC8]
```

| 字节 | 值 | 说明 |
| ---: | ---: | --- |
| 0 | `0xA5` | SOF_H |
| 1 | `0x5A` | SOF_L |
| 2 | `0x12` | CMD = CMD_GIMBAL_CALI_REQ |
| 3 | `0x01` | LEN = 1 |
| 4 | `0x01` | action = GIMBAL_CALI_REQ_START |
| 5 | `0x12` | CRC8 = 0x12 ^ 0x01 ^ 0x01 = 0x12 |

### 3.2 Python 示例代码

```python
import serial
import struct

def send_gimbal_cali_req(ser: serial.Serial):
    """发送云台校准请求帧"""
    SOF_H = 0xA5
    SOF_L = 0x5A
    CMD   = 0x12  # CMD_GIMBAL_CALI_REQ
    LEN   = 0x01
    ACTION = 0x01  # GIMBAL_CALI_REQ_START

    # CRC8 = CMD ^ LEN ^ payload
    crc = CMD ^ LEN ^ ACTION

    frame = struct.pack('6B', SOF_H, SOF_L, CMD, LEN, ACTION, crc)
    ser.write(frame)
    print(f"Sent gimbal cali request: {frame.hex()}")


def parse_status_report(data: bytes):
    """解析 CMD_STATUS_REPORT (0x81) payload (72 bytes)"""
    if len(data) < 72:
        return None
    hp = struct.unpack_from('<H', data, 0)[0]
    yaw_abs = struct.unpack_from('<f', data, 2)[0]
    pitch_abs = struct.unpack_from('<f', data, 6)[0]
    robot_id = data[10]
    shoot_speed_limit = data[11]
    last_nav_seq = struct.unpack_from('<H', data, 12)[0]
    last_nav_ts = struct.unpack_from('<I', data, 14)[0]
    nav_age_ms = struct.unpack_from('<H', data, 18)[0]
    status_flags = data[20]
    gate_state = data[61]

    return {
        'hp': hp,
        'yaw_abs_rad': yaw_abs,
        'pitch_abs_rad': pitch_abs,
        'robot_id': robot_id,
        'last_nav_seq': last_nav_seq,
        'last_nav_timestamp_ms': last_nav_ts,
        'nav_age_ms': nav_age_ms,
        'nav_fresh':       bool(status_flags & 0x01),
        'vision_online':   bool(status_flags & 0x02),
        'referee_online':  bool(status_flags & 0x04),
        'heat_blocked':    bool(status_flags & 0x08),
        'gimbal_hold':     bool(status_flags & 0x10),
        'gimbal_cali':     bool(status_flags & 0x20),
        'gimbal_cali_valid': bool(status_flags & 0x40),
        'gimbal_cali_running': bool(status_flags & 0x80),
        'dbg_gimbal_cali_request_count': struct.unpack_from('<I', data, 21)[0],
        'dbg_gimbal_cali_last_req_tick': struct.unpack_from('<I', data, 25)[0],
        'dbg_gimbal_cali_now_tick': struct.unpack_from('<I', data, 29)[0],
        'dbg_gimbal_cali_ready_tick': struct.unpack_from('<I', data, 33)[0],
        'dbg_gimbal_cali_ready_elapsed_ms': struct.unpack_from('<I', data, 37)[0],
        'dbg_gimbal_cali_yaw_age_ms': struct.unpack_from('<I', data, 41)[0],
        'dbg_gimbal_cali_pitch_age_ms': struct.unpack_from('<I', data, 45)[0],
        'dbg_gimbal_cali_imu_age_ms': struct.unpack_from('<I', data, 49)[0],
        'dbg_gimbal_cali_dependency_max_age_ms': struct.unpack_from('<I', data, 53)[0],
        'dbg_gimbal_cali_stable_time_ms': struct.unpack_from('<I', data, 57)[0],
        'dbg_gimbal_cali_gate_state': gate_state,
        'dbg_gimbal_cali_host_pending': bool(data[62]),
        'dbg_gimbal_cali_auto_pending': bool(data[63]),
        'dbg_gimbal_cali_pending': bool(data[64]),
        'dbg_gimbal_cali_running_raw': bool(data[65]),
        'dbg_gimbal_cali_valid_raw': bool(data[66]),
        'dbg_gimbal_cali_cmd': bool(data[67]),
        'dbg_gimbal_cali_block_nav_fresh': bool(data[68]),
        'dbg_gimbal_cali_yaw_recent': bool(data[69]),
        'dbg_gimbal_cali_pitch_recent': bool(data[70]),
        'dbg_gimbal_cali_imu_recent': bool(data[71]),
    }
```

### 3.3 ROS2 (C++) 发送示例

```cpp
#include <vector>
#include <cstdint>

std::vector<uint8_t> build_gimbal_cali_frame() {
    uint8_t cmd = 0x12;
    uint8_t len = 0x01;
    uint8_t action = 0x01;
    uint8_t crc = cmd ^ len ^ action;
    return {0xA5, 0x5A, cmd, len, action, crc};
}
```

---

## 4. 完整标定操作流程

### 步骤 1：上电并等待系统就绪

1. 给 STM32 上电
2. 等待约 2~3 秒，使 CAN 电机和 IMU 初始化完成
3. 上位机打开 UART1 串口（波特率按实际配置，通常 `115200` 或 `460800`）

### 步骤 2：确认云台当前状态

开始接收 `CMD_STATUS_REPORT (0x81)` 状态回传帧（每 50ms 一帧），检查：

| 检查项 | 期望值 | 说明 |
| --- | --- | --- |
| 能收到 `0x81` 帧 | 是 | 串口物理连接正常 |
| `gimbal_cali_valid` (bit6) | `0` | 说明需要标定 |
| `gimbal_cali` (bit5) | `0` 或 `1` | `1` = 正在等待或执行标定 |
| `gimbal_cali_running` (bit7) | `0` 或 `1` | `1` = 已真正进入扫位执行 |

如果条件允许，建议你再额外确认两件事：

1. `yaw` 电机上电后能被控制
2. `pitch` 电机在上电状态下，手拨时有一定抗力，或在标定触发时至少尝试动作

> 如果 `gimbal_cali_valid = 1`，说明 Flash 中已有有效标定数据，**无需再次标定**（除非想重标）。

### 步骤 3：停止发送 NAV 命令

**必须停止发送 `CMD_NAV_DATA`**。

原因：如果上位机持续发送 `CMD_NAV_DATA`，固件判定 `ros_nav_cmd_fresh() == true`，会一直推迟标定。

可以只发 `CMD_HEARTBEAT` 保持链路在线（可选）。

> 这是当前流程里最容易踩坑的地方：**标定期间不要继续发 yaw/pitch 目标帧。**
> 也就是说，“上位机参与标定”不等于“上位机持续发角度数据”。

### 步骤 4：发送标定请求

发送一帧 `CMD_GIMBAL_CALI_REQ(action=0x01)`：

```python
# 假设 ser 已打开
send_gimbal_cali_req(ser)
```

> 只需发一帧即可。固件内部会记住请求，满足条件后自动启动。

### 步骤 5：观察标定过程

标定启动后，你会观察到：

1. **蜂鸣器响起**（如果硬件连接了蜂鸣器）
2. **云台开始自动扫限位**：
   - Pitch 先向上（正向）旋转到极限 → 停下
   - Pitch 再向下（反向）旋转到极限 → 停下
   - Yaw 向一侧旋转到极限 → 停下
   - Yaw 向另一侧旋转到极限 → 停下
3. 每一步的检测逻辑是：电机以固定电流推动，当陀螺仪检测到角速度 < 0.1 rad/s 并持续约 2 秒时，判定为到达限位

当前固件还额外增加了两层 pitch 标定保护：

1. **pitch 必须先出现真实角度变化**，否则不会因为“角速度接近 0”而误判到位
2. 如果第一步 pitch 标定方向不对、短时间内几乎不动，固件会在**标定流程内部自动反一次 pitch 方向**再试一次

另外，针对你这套机构当前出现的“pitch 朝下垂方向使力”现象，当前固件已把 **pitch 标定默认原始电流方向** 独立成 `GIMBAL_CALI_PITCH_SIGN`，默认按本机现象设为反向，仅影响**标定 raw 扫位**，不影响正常闭环控制路径。

这样可以避免“pitch 实际没扫位，但流程已经跳去 yaw 一直左右转”的假成功现象。

总标定时间约 **8~12 秒**。

如果此时出现：

- yaw 会扫，pitch 完全不扫
- 蜂鸣器响，但 pitch 轴既不抗力也不动作

那么优先排查：

1. `pitch` 电机拨码 ID 是否真的是 `5`
2. `pitch` 这一路 CAN 支路是否接反 / 接触不良
3. `pitch` 机械是否卡死
4. `pitch` 电机/驱动是否损坏

如果你更新到当前版本后，仍然看到：

- pitch 不明显动作
- 但流程也**不会再继续跳去 yaw 长时间左右扫位**

那么这是符合预期的，说明固件已经识别出“pitch 这一步根本没真正扫到位”，问题继续收敛在 pitch 轴本体/接线/机械链路，而不是标定状态机乱跳步。

### 步骤 6：确认标定完成

持续读取状态回传：

```python
status = parse_status_report(payload)

# 标定等待中
if status['gimbal_cali'] and not status['gimbal_cali_running']:
    print("标定请求已登记，正在等待 ready 条件...")

# 标定真正执行中
if status['gimbal_cali_running']:
    print("标定扫位中...")

# 标定完成
if status['gimbal_cali_valid'] and not status['gimbal_cali']:
    print("标定成功！")
```

标定完成的标志：

| 状态位 | 值 | 含义 |
| --- | ---: | --- |
| `gimbal_cali` (bit5) | `0` | 不再处于等待/标定状态 |
| `gimbal_cali_valid` (bit6) | `1` | 有效标定数据已写入 Flash |
| `gimbal_cali_running` (bit7) | `0` | 当前已退出扫位执行阶段 |

### 步骤 7：恢复正常控制

标定完成后，恢复发送 `CMD_NAV_DATA`：

```python
def send_nav_data(ser, vx, vy, vz, yaw_abs, pitch_abs, flags, seq, ts_ms):
    """发送 CMD_NAV_DATA 帧"""
    SOF_H, SOF_L = 0xA5, 0x5A
    CMD, LEN = 0x02, 27

    # 完整 payload: vx(4) + vy(4) + vz(4) + yaw(4) + pitch(4) + flags(1) + seq(2) + ts(4) = 27
    payload = struct.pack('<fffffBHI', vx, vy, vz, yaw_abs, pitch_abs, flags, seq, ts_ms)

    crc = CMD ^ LEN
    for b in payload:
        crc ^= b

    frame = bytes([SOF_H, SOF_L, CMD, LEN]) + payload + bytes([crc & 0xFF])
    ser.write(frame)
```

---

## 5. 用上位机发送 yaw 数据来验证标定结果

标定完成后，可以通过发送不同的 `yaw_abs` / `pitch_abs` 来验证云台是否标定正确。

注意：验证阶段才需要持续发送角度目标；标定阶段不需要。

### 5.1 验证步骤

1. 读取当前状态回传中的 `yaw_abs` 和 `pitch_abs`（当前实际角度）
2. 发送一帧 `CMD_NAV_DATA`，目标角为当前角 + 一小段偏移（如 +0.1 rad ≈ 5.7°）
3. 观察云台是否朝预期方向移动
4. 观察状态回传中的角度是否趋近目标值

### 5.2 验证脚本示例

```python
import serial
import struct
import time
import math

PORT = '/dev/ttyUSB0'  # 根据实际修改
BAUD = 115200          # 根据实际修改

ser = serial.Serial(PORT, BAUD, timeout=0.1)

def build_nav_frame(vx, vy, vz, yaw, pitch, flags, seq):
    SOF_H, SOF_L = 0xA5, 0x5A
    CMD, LEN = 0x02, 27
    ts = int(time.monotonic() * 1000) & 0xFFFFFFFF

    payload = struct.pack('<fffffBHI', vx, vy, vz, yaw, pitch, flags, seq, ts)

    crc = CMD ^ LEN
    for b in payload:
        crc ^= b

    return bytes([SOF_H, SOF_L, CMD, LEN]) + payload + bytes([crc & 0xFF])

def parse_upstream(buf):
    """从缓冲区中查找并解析 0x81 帧"""
    idx = 0
    while idx < len(buf) - 1:
        if buf[idx] == 0xA5 and buf[idx+1] == 0x5A:
            if idx + 26 <= len(buf):
                cmd = buf[idx+2]
                plen = buf[idx+3]
                if cmd == 0x81 and plen == 21:
                    payload = buf[idx+4:idx+25]
                    return parse_status_report(payload), idx + 26
        idx += 1
    return None, len(buf)

# ----- 验证标定 -----

seq = 0

# 1. 先发一帧 gimbal-only 命令，读取当前角度
# flags = 0x02 -> NAV_FLAG_GIMBAL_ABS_VALID
frame = build_nav_frame(0.0, 0.0, 0.0, 0.0, 0.0, 0x02, seq)
ser.write(frame)
time.sleep(0.2)

raw = ser.read(256)
status, _ = parse_upstream(raw)
if status:
    print(f"当前角度: yaw={math.degrees(status['yaw_abs_rad']):.1f}°, "
          f"pitch={math.degrees(status['pitch_abs_rad']):.1f}°")
    print(f"标定有效: {status['gimbal_cali_valid']}")

    current_yaw = status['yaw_abs_rad']
    current_pitch = status['pitch_abs_rad']

    # 2. 发送 yaw 偏移 +10° 的目标
    target_yaw = current_yaw + math.radians(10.0)
    target_pitch = current_pitch  # pitch 不动

    print(f"\n发送目标: yaw={math.degrees(target_yaw):.1f}°, "
          f"pitch={math.degrees(target_pitch):.1f}°")

    for i in range(200):  # 持续发送 2 秒
        seq += 1
        frame = build_nav_frame(0.0, 0.0, 0.0,
                                target_yaw, target_pitch,
                                0x02, seq & 0xFFFF)
        ser.write(frame)
        time.sleep(0.01)

    # 3. 读取最终角度
    time.sleep(0.1)
    raw = ser.read(256)
    status, _ = parse_upstream(raw)
    if status:
        print(f"最终角度: yaw={math.degrees(status['yaw_abs_rad']):.1f}°, "
              f"pitch={math.degrees(status['pitch_abs_rad']):.1f}°")
        err = abs(status['yaw_abs_rad'] - target_yaw)
        print(f"Yaw 误差: {math.degrees(err):.2f}°")
        if err < math.radians(2.0):
            print("✓ 标定看起来正确")
        else:
            print("✗ 误差较大，需要检查标定")

ser.close()
```

---

## 6. 常见问题排查

### 6.1 我怀疑 pitch（上下）电机坏了，该怎么判断？

建议按下面顺序判断，不要只靠“灯亮不亮”或“断电能不能停住”：

#### 现象 A：断电后 pitch 轴不能停在任意位置

这**通常不是故障证据**，而是 GM6020 直驱结构的正常现象。

#### 现象 B：上电后 yaw 会动，pitch 不动，但 pitch 灯是亮的

这说明：

- 供电大概率没问题
- 但 **pitch 这路控制链** 仍可能有问题

优先排查顺序：

1. `pitch ID` 是否为 `5`
2. `pitch` 是否接在 `CAN2`
3. `pitch` 支路 CANH/CANL 是否接反
4. `pitch` 机械是否卡死
5. `pitch` 驱动是否已损坏

#### 现象 C：怀疑整条 CAN 线接反

如果 **yaw 正常而 pitch 不正常**，那么“整条 CAN2 从 C 板出来就接反”这件事**概率不高**，因为那样 yaw 通常也会一起不正常。

更像是：

- `pitch` 单独那一个接头 / 分支 / ID 有问题

### 6.2 标定一直不开始

| 可能原因 | 排查方法 |
| --- | --- |
| 上位机仍在发 `CMD_NAV_DATA` | 确认已停止发送 NAV 命令 |
| 电机未上线 | 检查 CAN2 接线和通信，观察电机灯 |
| IMU 解算链未刷新 | 确认 BMI088 正常工作，且 `RM_IMU_TOE` 被持续刷新 |
| Flash 中已有有效数据 | 检查 `gimbal_cali_valid` 是否已为 1 |
| pitch 电机 ID 不对 | 确认拨码与固件映射一致：pitch=5, yaw=6 |

进一步看状态位时建议这样判断：

- `gimbal_cali=1` 且 `gimbal_cali_running=0`：说明只是 **pending**，还没真正扫位
- `gimbal_cali=1` 且 `gimbal_cali_running=1`：说明已经真正开始标定

如果你已经确认：

- `CMD_GIMBAL_CALI_REQ(action=0x01)` 已发出
- 协议本身不想再改
- 但下位机仍表现为 **不响 / 不动 / 不进入 running**

那么当前版本已经额外提供了一组**下位机本地 watch 调试变量**，专门用来定位到底卡在哪个启动门控条件。

### 6.2.1 当前版本：调试变量已随 `CMD_STATUS_REPORT` 回传

这些变量定义在：

- `application/calibrate_task.h`
- `application/calibrate_task.c`

用途：

- 仍可供 **Keil Watch / Live Watch / JScope** 观察
- 现在也会通过 `CMD_STATUS_REPORT (0x81)` 一并回传
- 上位机可以直接解析，无需 ST-Link 才能看到 gate 卡点

推荐最先盯这几项：

| 变量名 | 含义 | 典型用途 |
| --- | --- | --- |
| `dbg_gimbal_cali_request_count` | 已登记的标定请求次数 | 判断 `CMD_GIMBAL_CALI_REQ` 是否真的落到下位机 |
| `dbg_gimbal_cali_last_req_tick` | 最近一次登记请求的系统 tick | 判断“刚才那一帧请求”有没有被接住 |
| `dbg_gimbal_cali_host_pending` | 主机触发 pending 标志 | 判断主机请求是否已挂起 |
| `dbg_gimbal_cali_auto_pending` | 自动标定 pending 标志 | 区分是主机触发还是上电自动触发 |
| `dbg_gimbal_cali_pending` | 总 pending 标志 | 判断是否还停在等待态 |
| `dbg_gimbal_cali_running` | 当前是否已进入 running | 判断是否真正开始扫位 |
| `dbg_gimbal_cali_valid` | 当前 Flash 标定是否有效 | 判断是不是因为已有有效数据而不再启动 |
| `dbg_gimbal_cali_gate_state` | 当前卡住的门控阶段枚举 | **最关键**，直接看卡在哪个门 |
| `dbg_gimbal_cali_block_nav_fresh` | 当前是否仍被 fresh NAV 挡住 | 排查“上位机其实还在发 NAV” |
| `dbg_gimbal_cali_yaw_recent` | yaw 反馈最近是否足够新鲜 | 排查 yaw 反馈刷新 |
| `dbg_gimbal_cali_pitch_recent` | pitch 反馈最近是否足够新鲜 | 排查 pitch 反馈刷新 |
| `dbg_gimbal_cali_imu_recent` | IMU 反馈最近是否足够新鲜 | 排查 IMU 解算刷新 |
| `dbg_gimbal_cali_yaw_age_ms` | yaw 最近反馈 age | 看离 20ms 门限差多少 |
| `dbg_gimbal_cali_pitch_age_ms` | pitch 最近反馈 age | 看离 20ms 门限差多少 |
| `dbg_gimbal_cali_imu_age_ms` | IMU 最近反馈 age | 看离 20ms 门限差多少 |
| `dbg_gimbal_cali_ready_elapsed_ms` | ready 稳定窗口已累计多久 | 判断是否卡在 100ms 稳定窗 |
| `dbg_gimbal_cali_dependency_max_age_ms` | 当前生效的 recent 门限 | 判断此刻按自动门限还是主机门限在判 |
| `dbg_gimbal_cali_stable_time_ms` | 当前生效的稳定窗门限 | 判断此刻需要累计多久 |

### 6.2.2 `dbg_gimbal_cali_gate_state` 对照表

| 值 | 宏名 | 含义 | 优先排查 |
| ---: | --- | --- | --- |
| 0 | `DBG_GIMBAL_CALI_GATE_IDLE` | 当前没有 pending 请求 | 先确认请求是否真正落地 |
| 1 | `DBG_GIMBAL_CALI_GATE_WAIT_START_DELAY` | 在等启动延迟 | 当前默认一般不会卡这里 |
| 2 | `DBG_GIMBAL_CALI_GATE_WAIT_NAV_CLEAR` | 还存在 fresh `CMD_NAV_DATA` | 上位机先停发 NAV |
| 3 | `DBG_GIMBAL_CALI_GATE_WAIT_YAW_FEEDBACK` | yaw 反馈不够新鲜 | 查 yaw CAN 反馈、ID、接线 |
| 4 | `DBG_GIMBAL_CALI_GATE_WAIT_PITCH_FEEDBACK` | pitch 反馈不够新鲜 | 查 pitch CAN 反馈、ID、接线 |
| 5 | `DBG_GIMBAL_CALI_GATE_WAIT_IMU_FEEDBACK` | IMU 解算输出不够新鲜 | 查 INS / BMI088 / `RM_IMU_TOE` |
| 6 | `DBG_GIMBAL_CALI_GATE_WAIT_STABLE_WINDOW` | 依赖都满足，但还在等稳定窗 | 看 `dbg_gimbal_cali_ready_elapsed_ms` 是否能涨到 100 |
| 7 | `DBG_GIMBAL_CALI_GATE_RUNNING` | 已经真正进入扫位 | 这时再去查后段执行逻辑 |
| 8 | `DBG_GIMBAL_CALI_GATE_VALID_READY` | 当前已有有效标定数据 | 不是“卡住”，而是已经有效 |

### 6.2.3 最快定位方法

如果现场现象是：

- 上位机发了标定命令
- 下位机不响、不动

那么优先看下面这个组合：

1. `dbg_gimbal_cali_request_count` 是否增加
2. `dbg_gimbal_cali_host_pending` 是否变成 `1`
3. `dbg_gimbal_cali_gate_state` 最终停在哪个值

可以直接这样理解：

- `request_count` 不变：请求根本没被下位机解析到
- `request_count` 增加、`host_pending=1`、`gate_state=2`：被 fresh NAV 挡住
- `gate_state=3/4/5`：被 yaw / pitch / IMU 新鲜度挡住
- `gate_state=6`：前置条件都过了，但稳定窗持续被打断
- `gate_state=7`：已经开始扫位，不再是启动门控问题

### 6.2.4 这次修掉的 detect 初始化问题，到底是什么

这次还一起修了一个很容易把人绕晕的启动期问题：

- 旧逻辑中，`detect_init()` 会把每个设备的 `new_time` 先初始化成当前 tick
- 如果只看 `now - new_time <= max_age`，那么设备在**还没有收到第一帧真实反馈**时，也可能在刚启动的一小段时间里被误判成“recent”
- 这就会造成一种错觉：第一次上电很早发请求，偶尔像是能开始；再发一次却又稳定卡 `pending`

当前版本的修正是：

- `detect_task` 增加 `data_updated` 标志
- 只有真实 `detect_hook()` 发生过，相关依赖才会被算作 recent

所以现在如果：

- 上电后不开上位机也不响
- `dbg_gimbal_cali_gate_state=3/4/5`

那就更可信地说明：

- yaw / pitch / IMU 里确实有某一路还没有收到真实新鲜反馈

而不是被 `detect_init()` 的初始时间戳“演了一出假 recent”。

如果你看到：

- `host_pending=1`
- `dbg_gimbal_cali_dependency_max_age_ms=100`
- `dbg_gimbal_cali_stable_time_ms=20`

说明当前已经在使用**主机触发的宽松门控**。如果此时仍然长期卡在 `3/4/5`，那就更像是真实链路问题，而不是门限过严导致的误杀。

### 6.3 标定完成但 yaw / pitch 方向相反

如果发送正向 yaw / pitch 角度但云台反转：

- 检查 [gimbal_behaviour.c](../application/gimbal_behaviour.c) 中的 `ROS_YAW_SIGN` 宏
- 当前默认 `ROS_YAW_SIGN = 1.0f`，若方向相反则改为 `-1.0f`
- 同理 pitch 方向对应 `ROS_PITCH_SIGN`

> 不建议同时改 `ROS_*_SIGN` 和 `YAW_TURN / PITCH_TURN`，否则很容易把问题“修成双重反相”。

### 6.4 标定完成但角度偏移很大

- 可能机械限位不对称或有干涉
- 标定过程中云台是否完整扫过了四个极限位
- 可通过上位机读取实际回传角度，对比预期值

### 6.5 标定结果校验失败（自动重试）

固件内部有效性校验要求：

- `yaw 最大角 - yaw 最小角 >= 0.6 rad`（约 34°）
- `pitch 最大角 - pitch 最小角 >= 0.25 rad`（约 14°）

如果不满足，标定会自动从头重试。

### 6.6 希望强制重新标定

即使 Flash 中已有有效数据，仍然可以发送 `CMD_GIMBAL_CALI_REQ(action=0x01)` 来强制触发一次新的标定。

### 6.7 一个推荐的排障闭环

建议按这个顺序做，最快：

1. 先确认 `pitch ID=5`、`yaw ID=6`
2. 确认两台 GM6020 都挂在 `CAN2`
3. 停止所有 `CMD_NAV_DATA`
4. 发一次 `CMD_GIMBAL_CALI_REQ`
5. 看是否出现 **pitch 先动、再 yaw 动** 的标定序列
6. 若 pitch 始终不动，优先查 pitch 分支接线/ID/电机本体
7. 若标定成功，再用 `yaw_abs / pitch_abs` 连续发送做闭环验证

### 6.8 现场联调观察表（推荐直接照表看）

下面这张表是给上下位机联调时直接用的。建议上位机每 50ms 左右打印一次：

- `nav_fresh`
- `nav_age_ms`
- `gimbal_cali`
- `gimbal_cali_running`
- `gimbal_cali_valid`

然后按下表判断：

| `nav_fresh` | `gimbal_cali` | `gimbal_cali_running` | `gimbal_cali_valid` | 当前含义 | 优先动作 |
| ---: | ---: | ---: | ---: | --- | --- |
| 1 | 0/1 | 0 | 0/1 | 仍有新鲜 `CMD_NAV_DATA`，标定启动门会被挡住 | 先停发 `CMD_NAV_DATA`，只保留心跳或完全静默 |
| 0 | 1 | 0 | 0 | 标定请求已登记，但还卡在 **pending** | 查 yaw/pitch 反馈、IMU 解算刷新、ready 条件是否连续满足 |
| 0 | 1 | 1 | 0 | 已真正进入 **running**，正在扫位 | 观察是否出现 `Pitch -> Yaw` 扫位顺序 |
| 0 | 0 | 0 | 1 | 标定成功，Flash 中已有有效数据 | 恢复发送 `CMD_NAV_DATA` 做姿态闭环验证 |
| 0 | 0 | 0 | 0 | 当前既没在标定，也没有有效标定数据 | 重新发 `CMD_GIMBAL_CALI_REQ`，并继续查 ready 门控 |
| 0 | 1 | 1 | 1 | 极少见，通常是运行结束前的瞬态 | 继续观察，正常应很快回到 `0/0/1` |

进一步结合现场现象，可以这样缩小范围：

| 状态组合 | 机械现象 | 更像是什么问题 |
| --- | --- | --- |
| `0 / 1 / 0 / 0` | 完全不扫位 | 还卡在下位机 ready 门控 |
| `0 / 1 / 1 / 0` | yaw 扫、pitch 不扫 | pitch 电机链路 / 机械本体问题 |
| `0 / 1 / 1 / 0` | pitch、yaw 都不扫 | 进入 running 了，但执行层控制链异常 |
| `0 / 0 / 0 / 1` | 不扫位但能正常保持 | 不是故障，说明已存在有效标定数据 |

> 记忆口诀：
>
> - `bit5=1, bit7=0`：**在等，不是在扫**
> - `bit5=1, bit7=1`：**真扫了**
> - `bit6=1`：**标定数据已经有效**

---

## 7. 标定数据存储

- 标定数据存储在 STM32 内部 Flash 的 **Sector 9**（地址 `0x080A0000`）
- 包含：`yaw_offset_ecd`, `pitch_offset_ecd`, `yaw_max_angle`, `yaw_min_angle`, `pitch_max_angle`, `pitch_min_angle`
- 标定完成后，`cali_flag = 0x55`，表示数据有效
- 下次上电时固件会自动读取并应用，不再触发标定

---

## 8. 整体流程时序图

```text
上位机 (ROS)                              STM32 (UART1)
    |                                         |
    |  [上电初始化 ~2s]                        |
    |                                         |
    |<--- CMD_STATUS_REPORT (0x81) -----------|  (gimbal_cali_valid=0)
    |                                         |
    |--- CMD_GIMBAL_CALI_REQ (0x12) --------->|  (action=0x01)
    |                                         |
    |     [停止发送 CMD_NAV_DATA]               |
    |                                         |
    |<--- CMD_STATUS_REPORT ------------------|  (gimbal_cali=1)
    |                                         |
    |     [云台开始自动扫限位 ~10s]              |
    |     Pitch↑ → Pitch↓ → Yaw→ → Yaw←       |
    |                                         |
    |<--- CMD_STATUS_REPORT ------------------|  (gimbal_cali=0, gimbal_cali_valid=1)
    |                                         |
    |--- CMD_NAV_DATA (0x02) ---------------->|  (恢复控制)
    |                                         |
```
