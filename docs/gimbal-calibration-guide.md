# 云台标定流程文档（无遥控器 / 上位机触发版）

本文档介绍如何在**没有遥控器**的情况下，通过上位机（ROS / Ubuntu）发送 UART1 指令来完成云台标定。

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

### 1.3 自动标定

固件默认启用了上电自动标定（`AUTO_GIMBAL_CALI_ENABLE = 1`）：

- 当 Flash 中没有有效标定数据时
- 等待 yaw/pitch 电机和 IMU 全部上线
- 且当前没有新鲜 `CMD_NAV_DATA`（即上位机没在发控制命令）
- 稳定等待约 100ms 后自动开始

> **如果自动标定已经成功完成并写入 Flash，后续上电将跳过标定直接进入正常控制。**

---

## 2. 前提条件

在触发标定前，请确认以下条件：

| 条件 | 说明 |
| --- | --- |
| CAN 总线正常 | yaw / pitch 电机在 CAN2 上正常通信 |
| 电机已上线 | 上电后 `YAW_GIMBAL_MOTOR_TOE` 和 `PITCH_GIMBAL_MOTOR_TOE` 已在线 |
| IMU 已上线 | 板载 IMU (`RM_IMU_TOE`) 已在线 |
| 无新鲜 NAV 命令 | 上位机**停止发送** `CMD_NAV_DATA`（否则标定被推迟） |
| 云台可自由旋转 | 机械上 yaw / pitch 都能安全地转到极限位置再弹回来 |

---

## 3. 上位机发送标定指令

### 3.1 帧格式

使用 `CMD_GIMBAL_CALI_REQ = 0x12`，payload 长度 1 byte。

完整帧（6 bytes）：

```
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
    """解析 CMD_STATUS_REPORT (0x81) payload (21 bytes)"""
    if len(data) < 21:
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

    return {
        'hp': hp,
        'yaw_abs_rad': yaw_abs,
        'pitch_abs_rad': pitch_abs,
        'robot_id': robot_id,
        'nav_fresh':       bool(status_flags & 0x01),
        'vision_online':   bool(status_flags & 0x02),
        'referee_online':  bool(status_flags & 0x04),
        'heat_blocked':    bool(status_flags & 0x08),
        'gimbal_hold':     bool(status_flags & 0x10),
        'gimbal_cali':     bool(status_flags & 0x20),
        'gimbal_cali_valid': bool(status_flags & 0x40),
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
| `gimbal_cali` (bit5) | `0` 或 `1` | `1` = 正在等待/执行自动标定 |

> 如果 `gimbal_cali_valid = 1`，说明 Flash 中已有有效标定数据，**无需再次标定**（除非想重标）。

### 步骤 3：停止发送 NAV 命令

**必须停止发送 `CMD_NAV_DATA`**。

原因：如果上位机持续发送 `CMD_NAV_DATA`，固件判定 `ros_nav_cmd_fresh() == true`，会一直推迟标定。

可以只发 `CMD_HEARTBEAT` 保持链路在线（可选）。

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

总标定时间约 **8~12 秒**。

### 步骤 6：确认标定完成

持续读取状态回传：

```python
status = parse_status_report(payload)

# 标定进行中
if status['gimbal_cali']:
    print("标定中...")

# 标定完成
if status['gimbal_cali_valid'] and not status['gimbal_cali']:
    print("标定成功！")
```

标定完成的标志：

| 状态位 | 值 | 含义 |
| --- | ---: | --- |
| `gimbal_cali` (bit5) | `0` | 不再处于标定状态 |
| `gimbal_cali_valid` (bit6) | `1` | 有效标定数据已写入 Flash |

### 步骤 7：恢复正常控制

标定完成后，恢复发送 `CMD_NAV_DATA`：

```python
def send_nav_data(ser, vx, vy, vz, yaw_abs, pitch_abs, flags, seq, ts_ms):
    """发送 CMD_NAV_DATA 帧"""
    SOF_H, SOF_L = 0xA5, 0x5A
    CMD, LEN = 0x02, 27

    payload = struct.pack('<fffBHI',
                          yaw_abs, pitch_abs,  # ... 完整打包
                          flags, seq, ts_ms)
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

# 1. 先发一帧零命令，读取当前角度
frame = build_nav_frame(0.0, 0.0, 0.0, 0.0, 0.0, 0x01, seq)
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
                                0x01, seq & 0xFFFF)
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

### 6.1 标定一直不开始

| 可能原因 | 排查方法 |
| --- | --- |
| 上位机仍在发 `CMD_NAV_DATA` | 确认已停止发送 NAV 命令 |
| 电机未上线 | 检查 CAN2 接线和通信，观察电机灯 |
| IMU 未上线 | 确认 SPI 连接的 BMI088 正常工作 |
| Flash 中已有有效数据 | 检查 `gimbal_cali_valid` 是否已为 1 |

### 6.2 标定完成但 yaw 方向相反

如果发送正向 yaw 角度但云台反转：

- 检查 [gimbal_behaviour.c](../application/gimbal_behaviour.c) 中的 `ROS_YAW_SIGN` 宏
- 当前默认 `ROS_YAW_SIGN = 1.0f`，若方向相反则改为 `-1.0f`
- 同理 pitch 方向对应 `ROS_PITCH_SIGN`

### 6.3 标定完成但角度偏移很大

- 可能机械限位不对称或有干涉
- 标定过程中云台是否完整扫过了四个极限位
- 可通过上位机读取实际回传角度，对比预期值

### 6.4 标定结果校验失败（自动重试）

固件内部有效性校验要求：

- `yaw 最大角 - yaw 最小角 >= 0.6 rad`（约 34°）
- `pitch 最大角 - pitch 最小角 >= 0.25 rad`（约 14°）

如果不满足，标定会自动从头重试。

### 6.5 希望强制重新标定

即使 Flash 中已有有效数据，仍然可以发送 `CMD_GIMBAL_CALI_REQ(action=0x01)` 来强制触发一次新的标定。

---

## 7. 标定数据存储

- 标定数据存储在 STM32 内部 Flash 的 **Sector 9**（地址 `0x080A0000`）
- 包含：`yaw_offset_ecd`, `pitch_offset_ecd`, `yaw_max_angle`, `yaw_min_angle`, `pitch_max_angle`, `pitch_min_angle`
- 标定完成后，`cali_flag = 0x55`，表示数据有效
- 下次上电时固件会自动读取并应用，不再触发标定

---

## 8. 整体流程时序图

```
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
