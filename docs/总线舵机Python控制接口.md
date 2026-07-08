# 总线舵机 Python 控制接口

本文件说明 `project/src/bus_servo.py` 中的直接串口控制接口。实现依据 `02 总线舵机通信协议.pdf`，默认串口速率为 `115200`。

## 协议要点

- 物理层：半双工 UART，默认波特率 `115200bps`。
- 帧格式：`55 55 ID Length Cmd Params... Checksum`。
- `Length = 参数字节数 + 3`。
- `Checksum = ~(ID + Length + Cmd + Params...) & 0xFF`。
- 普通舵机 ID 范围：`0..253`，广播 ID：`254`。

## 已实现命令

| 功能 | 命令号 | 接口 |
|---|---:|---|
| 读取温度 | `26` | `read_temperature(servo_id)` |
| 读取角度/位置 | `28` | `read_position(servo_id)` |
| 读取电压 | `27` | `read_voltage(servo_id)` |
| 一次读取状态 | 温度 + 位置 + 电压 | `read_status(servo_id)` / `read_servo_status(com_port, servo_id)` |
| Servo 位置模式 | `1` | `move_servo(servo_id, target_position, run_time_ms)` / `set_servo_position(...)` |
| Motor 连续旋转模式 | `29` | `set_motor_speed(...)` |

## 当前机械臂控制约定

当前运动规划方案中，J1-J5 使用 servo 模式，J6 夹爪使用 motor 模式。

home 位置：

```text
J1=500, J2=478, J3=641, J4=890, J5=500
```

J6 抓取策略：

| 动作 | 参数 |
|---|---|
| home | J6 保持当前状态，home 时不发 J6 信号 |
| 夹取 | motor 速度 `100` |
| 夹到判定 | 夹取时持续读取 J6 位置，连续 `2s` 位置不变认为夹到 |
| 夹紧 | 判定夹到后将 motor 速度提高到 `300` |
| 张开 | motor 速度 `-100` |

J6 真实位置范围仍为 `0..1000`：`0` 为完全张开，`700` 为几何闭合，`700..1000` 只增加夹持力。

## 函数接口

```python
from project.src.bus_servo import read_servo_status, set_servo_position, set_motor_speed

# 读取状态：温度、当前位置、电压
status = read_servo_status("COM3", 1)
print(status.temperature_c, status.position, status.voltage_mv)

# Servo 模式：ID 1，移动到 500，运行 1000ms
set_servo_position("COM3", 1, target_position=500, run_time_ms=1000)

# Motor 模式：ID 1，占空比速度 100
set_motor_speed("COM3", 1, speed=100)
```

## 类接口

```python
from project.src.bus_servo import BusServoController

with BusServoController("COM3") as servo:
    status = servo.read_status(1)
    servo.move_servo(1, target_position=500, run_time_ms=1000)
    servo.set_motor_speed(1, speed=100)
```

## 命令行调试

```powershell
python project/src/bus_servo.py status COM3 1
python project/src/bus_servo.py servo COM3 1 500 1000
python project/src/bus_servo.py motor COM3 1 100
```

## 参数范围

| 参数 | 范围 | 说明 |
|---|---:|---|
| `servo_id` | `0..253` | 单个舵机 ID |
| `target_position` | `0..1000` | 对应约 `0..240°` |
| `run_time_ms` | `0..30000` | 运行时间，单位 ms |
| `speed` | `-1000..1000` | 默认 motor 占空比模式 |

如需使用固定速度模式，可调用类接口：

```python
with BusServoController("COM3") as servo:
    servo.set_motor_speed(1, speed=20, fixed_speed_mode=True)
```

固定速度模式速度范围为 `-50..50`。

## 依赖

实际连接硬件时需要安装：

```powershell
pip install pyserial
```

单元测试不依赖真实串口，会使用模拟串口验证帧格式、校验和、状态解析。
