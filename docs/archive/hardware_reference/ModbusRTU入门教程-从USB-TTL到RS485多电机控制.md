> 历史归档：原路径 `hardware/tempp/ModbusRTU入门教程-从USB-TTL到RS485多电机控制.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# Modbus RTU 入门教程：从 USB-TTL 到 RS485 多电机控制

> 适用读者：已使用因时自定义协议（USB 转 TTL）控制单个电机，希望迁移到 Modbus RTU + RS485 总线控制多个电机。
>
> 目标硬件：因时 AED-LA-92-12MR1 Modbus RTU 转换器 + LAF 系列微型伺服电缸
>
> 编写日期：2026-06-10

---

## 目录

1. [你现在的方案 vs. Modbus 方案（先看懂区别）](#1-你现在的方案-vs-modbus-方案)
2. [Modbus 是什么？三句话讲清楚](#2-modbus-是什么三句话讲清楚)
3. [Modbus RTU 通信参数配对](#3-modbus-rtu-通信参数配对)
4. [Modbus RTU 数据帧：你的"新语言"](#4-modbus-rtu-数据帧你的新语言)
5. [因时转换器寄存器速查表](#5-因时转换器寄存器速查表)
6. [硬件接线：从一根杜邦线到一根双绞线](#6-硬件接线)
7. [上电第一步：改波特率 + 设从站地址](#7-上电第一步改波特率--设从站地址)
8. [用 Python 控制电机：从 pyserial 到 minimalmodbus](#8-用-python-控制电机从-pyserial-到-minimalmodbus)
9. [常用操作速查（对照表）](#9-常用操作速查)
10. [常见错误与排查](#10-常见错误与排查)
11. [从单电机到 6 电机：多从站轮询](#11-从单电机到-6-电机多从站轮询)

---

## 1. 你现在的方案 vs. Modbus 方案

### 你现在的做法（USB-TTL + 因时自定义协议）

```
[你的 Python 代码]
       │
       ▼
  pyserial (串口库)
       │
       ▼
  USB 转 TTL 模块 (CH340/CP2102)
       │  4根杜邦线: GND/3.3V/TX/RX
       ▼
  因时电机 (LVTTL 3.3V 串口)
```

你做的事情（以你 GitHub 项目 `inspire_motor_control` 为例）：

```python
import serial

# 1. 打开串口
ser = serial.Serial('/dev/ttyUSB0', baudrate=921600, timeout=0.05)

# 2. 按照因时的帧格式拼一包数据，发过去
frame = bytes([0x55, 0xAA, 0x08, 0x01, 0x04, 0x24, 0x00, 0xD0, 0x07, 0x5E])
ser.write(frame)

# 3. 读应答
response = ser.read(20)

# 4. 自己按字节解析应答，校验 checksum
```

**特点：** 你自己的代码负责拼帧、校验、解析。一个串口只能接一个电机。

### Modbus RTU 方案（RS485 + Modbus 协议）

```
[你的 Python 代码]
       │
       ▼
  minimalmodbus (Modbus 库)
       │
       ▼
  USB 转 RS485 模块 (CH340+SP3485)
       │  RS485 总线 (A/B/GND 三线)
       ├──── AED-LA-92-12MR1 #1 ──── 电机 #1
       ├──── AED-LA-92-12MR1 #2 ──── 电机 #2
       └──── AED-LA-92-12MR1 #3 ──── 电机 #3
         (同一根总线，多个电机！)
```

你做的事情：

```python
import minimalmodbus

# 1. 创建一个 instrument，指定从站地址
motor1 = minimalmodbus.Instrument('/dev/ttyUSB0', slaveaddress=1)
motor1.serial.baudrate = 115200
motor1.serial.bytesize = 8
motor1.serial.parity = minimalmodbus.serial.PARITY_EVEN
motor1.serial.stopbits = 1

# 2. 直接写寄存器——不用自己拼帧！
motor1.write_register(0x0020, 1000)   # 电机1 走到 1000 步

# 3. 直接读寄存器——不用自己解帧！
position = motor1.read_register(0x0021)  # 读当前位置
```

**特点：** Modbus 库帮你处理拼帧、校验、解析。你只需要关心"往哪个寄存器写什么值"。多个电机共享同一个串口，通过从站地址区分。

### 一句话总结变化

| | 你现在 | Modbus 方案 |
|---|---|---|
| 物理层 | LVTTL 点对点 | RS485 总线（可并联多设备） |
| 拼帧 | 自己拼 `[0x55, 0xAA, ...]` | 库自动处理 |
| 校验 | 自己算 checksum | 库自动算 CRC |
| 多电机 | 6 个 USB 口 | 1 个 USB 口 + RS485 总线 |
| 协议 | 因时私有协议 | Modbus RTU 标准协议 |

---

## 2. Modbus 是什么？三句话讲清楚

1. **Modbus 是一种"问答式"通信协议**：主机（你的电脑）提问，从机（电机转换器）回答。从机不会主动说话。

2. **Modbus RTU 是 Modbus 的一种传输模式**：用二进制字节通信（区别于 Modbus ASCII 用可读字符）。RTU 模式更紧凑、更快，是工业界最常用的。

3. **核心概念是"寄存器"**：把从机看作一个 Excel 表格。你想让电机走到 1000 步，就是往"目标位置"那个单元格（寄存器地址 0x0020）写入 1000。你想知道电机当前位置，就是读"当前位置"那个单元格（寄存器地址 0x0021）。

**类比理解：**

```
Modbus 就像你给电机发短信——

你：  "从站1号，请把 0x0020 号寄存器的值设成 1000"
     ↓ (RS485 总线)
转换器#1： "收到，已设好"
     ↑ (RS485 总线)
```

---

## 3. Modbus RTU 通信参数配对

这是**最容易踩坑**的地方。主机（你的代码）和所有从机（所有转换器）必须在以下参数上完全一致：

| 参数 | 因时转换器默认值 | 说明 |
|------|-----------------|------|
| 波特率 (Baud Rate) | **9600** | 出厂默认，强烈建议改为 115200 |
| 数据位 (Data Bits) | **8** | 固定值，几乎所有设备都是 8 |
| 校验位 (Parity) | **Even（偶校验）** | 因时的设定，注意不是 None！ |
| 停止位 (Stop Bits) | **1** | 固定值 |
| 从站地址 (Slave Address) | **1**（出厂） | 每个转换器必须唯一，范围 1~254 |

**关键：** 这 5 个参数只要有一个不匹配，通信就失败（现象：发指令没反应，超时报错）。

### 你的代码和电机必须像钥匙和锁一样配对

```
你的代码                   因时转换器
─────────                 ──────────
baudrate = 115200   ←→   波特率 = 115200
bytesize = 8        ←→   数据位 = 8
parity = EVEN       ←→   校验 = 偶校验
stopbits = 1        ←→   停止位 = 1
slaveaddress = 1    ←→   从站地址 = 1
```

---

## 4. Modbus RTU 数据帧：你的"新语言"

### 你不必手动拼帧！

用 minimalmodbus 库时，你不需要手动构造这些帧。但理解帧结构有助于调试。下面用一个实际例子来讲。

### 例子：让 1 号电机走到位置 1000

你要操作的寄存器：`CMD-SETPOS`，地址 `0x0020`，写入值 `1000`（即 `0x03E8`）

**你的 Python 代码（一行搞定）：**

```python
motor.write_register(0x0020, 1000)
```

**库在背后自动发出的 Modbus RTU 帧（8 个字节）：**

```
从站地址  功能码   寄存器地址    写入数据      CRC 校验
┌──────┐ ┌────┐ ┌─────┬─────┐ ┌─────┬─────┐ ┌─────┬─────┐
│  01  │ │ 06 │ │ 00  │ 20  │ │ 03  │ E8  │ │ 89  │ 5D  │
└──────┘ └────┘ └─────┴─────┘ └─────┴─────┘ └─────┴─────┘
   ↑       ↑        ↑              ↑              ↑
  1号电机  写单个   寄存器0x0020   值=1000       CRC16校验
          寄存器                            （库自动计算）
```

**转换器回复的帧：**

```
┌──────┐ ┌────┐ ┌─────┬─────┐ ┌─────┬─────┐ ┌─────┬─────┐
│  01  │ │ 06 │ │ 00  │ 20  │ │ 03  │ E8  │ │ 89  │ 5D  │
└──────┘ └────┘ └─────┴─────┘ └─────┴─────┘ └─────┴─────┘
  (回复和请求一样，表示写入成功)
```

### Modbus 功能码速查

因时转换器支持的功能码：

| 功能码 | 名称 | 用途 | 对应 Python 方法 |
|--------|------|------|------------------|
| **0x03** | Read Holding Registers | 读一个或多个寄存器 | `read_register()` / `read_registers()` |
| **0x06** | Write Single Register | 写一个寄存器 | `write_register()` |
| **0x10** | Write Multiple Registers | 一次写多个寄存器 | `write_registers()` |

### 对比：你原来因时协议的帧 vs. Modbus 帧

```
【因时自定义协议】
  0x55 0xAA | Length | ID | CMD | RegAddr(LE) | Data(LE) | Checksum
  帧头两个字节  长度    ID  命令    寄存器地址      数据      校验和

【Modbus RTU】
  SlaveAddr | FuncCode | RegAddr(BE) | Data(BE) | CRC16(LE)
  从站地址    功能码     寄存器地址     数据       CRC校验
```

关键区别：
- Modbus 用 **CRC16** 而非简单 Checksum（更可靠）
- Modbus 数据是 **大端序（Big-Endian）**，因时原生协议是小端序
- Modbus 帧是**标准格式**，任何 Modbus 调试工具都能解析

---

## 5. 因时转换器寄存器速查表

这是你日常操作最常用的参考表。打印出来贴在显示器旁边。

### 动作类寄存器（写）

| 寄存器 | 名称 | 写什么 | 效果 |
|--------|------|--------|------|
| **0x0020** | CMD-SETPOS | 0 ~ 2000 | 电机走到指定位置（步） |
| **0x0014** | CMD-SETVEL | 1 ~ Vmax | 设定速度（步/秒） |
| **0x001F** | CMD-SETFORCE | -15000 ~ +15000 | 设定力控目标值（g） |
| **0x0010** | CMD-STOP | 1 | 急停！ |
| 0x0011 | CMD-RESTART | 1 | 恢复运行 |
| 0x0012 | CMD-FAULTACK | 1 | 清除故障 |

### 状态类寄存器（读）

| 寄存器 | 名称 | 读到的是 | 备注 |
|--------|------|---------|------|
| **0x0021** | CMD_CURPOS | -100 ~ 2100 | 电机当前位置 |
| 0x0022 | CMD_CURTEMP | -20 ~ 100 | 当前温度（℃） |
| 0x0023 | CMD_CURCUR | 0 ~ 2000 | 当前电流（mA） |
| 0x0024 | CMD_CURERR | 故障码 | 0=正常，1=堵转，2=过温，4=过流，8=电机异常 |
| 0x0025 | CMD_FORCE | -15000 ~ +15000 | 当前受力值（g） |

### 配置类寄存器

| 寄存器 | 名称 | 说明 |
|--------|------|------|
| 0x0001 | CMD-ID | 从站地址（1~254），改后立即生效 |
| 0x0002 | CMD-BAUD | 波特率，改后需断电重启 |
| 0x0003 | CMD-SAVE | 写 1 保存当前所有参数到 Flash |

### 重点记忆（最常用的 3 个寄存器）

```
0x0020  写位置    →  motor.write_register(0x0020, 1000)
0x0021  读位置    →  motor.read_register(0x0021)
0x0010  急停      →  motor.write_register(0x0010, 1)
```

---

## 6. 硬件接线

### 物料清单（最小验证系统，先控制 1 个电机）

| 序号 | 器材 | 数量 |
|------|------|------|
| 1 | 因时 AED-LA-92-12MR1 转换器 | 1 |
| 2 | USB 转 RS485 模块（推荐 CH340+SP3485） | 1 |
| 3 | 因时 LAF50-024D 电缸 | 1 |
| 4 | 24V DC 电源适配器（≥1A） | 1 |
| 5 | 杜邦线 母对母 4 根 | 4 |

> 等单电机跑通后，扩展为 6 电机只需：加 5 个转换器 + 换 ≥6A 的 24V 电源。

### 接线图（重点！接错会烧模块）

```
                ┌────────────────────────────────┐
                │   AED-LA-92-12MR1 转换器        │
                │                                │
   USB-RS485    │  左侧(RS485)      右侧(电缸)     │
   ┌──────┐     │  ┌─────────┐    ┌──────────┐   │
   │  A+  ├─────┼──┤ A+      │    │ GND(黑)  ├───┼── 电缸 pin1 (GND)
   │  B-  ├─────┼──┤ B-      │    │ 8V(红)   ├───┼── 电缸 pin2 (VCC)
   │ GND  ├─────┼──┤ GND     │    │ TX(黄)   ├───┼── 电缸 pin3 (RXD)
   └──────┘     │  │ 24V_in  │    │ RX(蓝)   ├───┼── 电缸 pin4 (TXD)
                │  └────┬───┘    └──────────┘   │
  24V电源       │       │                        │
  ┌──────┐     │       │                        │
  │ 24V+ ├─────┼───────┘                        │
  │ GND  ├─────┼── 与 RS485 的 GND 共地          │
  └──────┘     └────────────────────────────────┘
```

**严禁接反！接反即烧！接线前三遍确认：**
1. 电缸 pin1(黑) → 转换器 GND（黑），不是其他颜色！
2. 电缸 pin2(红) → 转换器 8V_out（红）
3. 电缸 pin3(黄) → 转换器 TX（黄）
4. 电缸 pin4(蓝) → 转换器 RX（蓝）

> 因时电缸的 D 型 4pin 接口定义见电气手册 PRJ-01-TS-U-007。颜色对应关系为因时原厂线缆标准。

---

## 7. 上电第一步：改波特率 + 设从站地址

出厂默认波特率是 **9600**（太慢），必须先改成 **115200**。

### 7.1 用 Modbus 调试工具改（推荐先用 GUI 验证通信）

推荐工具：**Modbus Poll**（Windows，有试用版）或 **QModMaster**（免费开源）

步骤：
1. USB-RS485 模块插入电脑，确认串口号（如 COM3）
2. 打开 Modbus Poll，设置：
   - 串口：COM3
   - 波特率：**9600**（出厂默认，先别改）
   - 数据位：8
   - 校验：Even
   - 停止位：1
3. 连接后，读寄存器 0x0021（CMD_CURPOS），看能否读到当前位置值
4. 如果能读到，说明通信正常 → 写入 0x0002 = 0x000A（115200）来改波特率
5. 写入 0x0003 = 1 保存参数
6. **断电重启**
7. 把 Modbus Poll 的波特率也改为 115200，重新连接
8. 再次读寄存器 0x0021，确认正常

### 7.2 改从站地址（每个转换器设不同的地址）

出厂默认从站地址都是 **1**，多电机时必须改成不同值：

```
转换器#1 → 地址 1
转换器#2 → 地址 2   （写入 0x0001 = 2）
转换器#3 → 地址 3   （写入 0x0001 = 3）
...
```

---

## 8. 用 Python 控制电机：从 pyserial 到 minimalmodbus

### 8.1 安装

```bash
pip install minimalmodbus pyserial
```

### 8.2 你现在写的代码 vs. Modbus 版本对照

**你现有的写法（因时自定义协议）：**

```python
# === 当前你的做法 ===
import serial
from insmotor_driver.protocol import (
    pack_read_status_cmd,
    pack_write_register_cmd,
    unpack_status_response,
    Register,
    ControlMode,
)

ser = serial.Serial('/dev/ttyUSB0', baudrate=921600, timeout=0.05)

# 读状态
frame = pack_read_status_cmd(motor_id=1)
ser.write(frame)
response = ser.read(128)
status = unpack_status_response(response)
print(f"位置: {status.actual_position}, 电流: {status.current_mA}")

# 走到位置 1000
frame = pack_write_register_cmd(motor_id=1, addr=Register.POSITION_TARGET, data=[1000])
ser.write(frame)
response = ser.read(20)
ser.close()
```

**Modbus 版本的写法：**

```python
# === Modbus 版本 ===
import minimalmodbus

# 创建 instrument（相当于打开串口 + 指定电机ID）
motor = minimalmodbus.Instrument('/dev/ttyUSB0', slaveaddress=1)
motor.serial.baudrate = 115200        # 注意：不再是 921600！
motor.serial.bytesize = 8
motor.serial.parity = minimalmodbus.serial.PARITY_EVEN
motor.serial.stopbits = 1
motor.serial.timeout = 0.1            # 100ms 超时

# 读状态（对应于 CMD_CURPOS=0x0021, CMD_CURCUR=0x0023, CMD_CURERR=0x0024）
position = motor.read_register(0x0021)       # 当前位置
current_ma = motor.read_register(0x0023)     # 当前电流
error_code = motor.read_register(0x0024)     # 故障码
print(f"位置: {position}, 电流: {current_ma}mA, 故障: {error_code}")

# 走到位置 1000
motor.write_register(0x0020, 1000)

motor.serial.close()
```

### 8.3 完整的单电机控制脚本（开箱即用）

```python
#!/usr/bin/env python3
"""
因时电缸 Modbus RTU 控制 —— 单电机最小示例
硬件：AED-LA-92-12MR1 + LAF50-024D
"""
import time
import minimalmodbus


class ModbusMotor:
    """因时电缸 Modbus RTU 控制封装"""

    # 寄存器地址
    REG_SETPOS   = 0x0020   # 目标位置    (R/W)
    REG_CURPOS   = 0x0021   # 当前位置    (R)
    REG_CURTEMP  = 0x0022   # 当前温度    (R)
    REG_CURCUR   = 0x0023   # 当前电流    (R)
    REG_CURERR   = 0x0024   # 故障码      (R)
    REG_CURFORCE = 0x0025   # 实际受力    (R)
    REG_SETVEL   = 0x0014   # 速度控制    (R/W)
    REG_SETFROCE = 0x001F   # 力控目标    (R/W)
    REG_STOP     = 0x0010   # 急停        (W)
    REG_RESTART  = 0x0011   # 恢复运行    (W)
    REG_FAULTACK = 0x0012   # 清除故障    (W)

    def __init__(self, port: str, slave_address: int = 1):
        self.addr = slave_address
        self.inst = minimalmodbus.Instrument(port, slaveaddress=slave_address)
        self.inst.serial.baudrate = 115200
        self.inst.serial.bytesize = 8
        self.inst.serial.parity = minimalmodbus.serial.PARITY_EVEN
        self.inst.serial.stopbits = 1
        self.inst.serial.timeout = 0.1

    # ── 运动控制 ──────────────────────────────────

    def move_to(self, position: int):
        """走到目标位置（定位模式），position: 0~2000 步"""
        position = max(0, min(2000, position))
        self.inst.write_register(self.REG_SETPOS, position)

    def set_speed(self, speed: int):
        """设置运动速度，speed: 步/秒"""
        self.inst.write_register(self.REG_SETVEL, speed)

    def set_force(self, force_target: int):
        """设置力控目标值（力控型），force_target: -15000~+15000 g"""
        self.inst.write_register(self.REG_SETFROCE, force_target)

    def emergency_stop(self):
        """急停"""
        self.inst.write_register(self.REG_STOP, 1)

    def restart(self):
        """恢复运行"""
        self.inst.write_register(self.REG_RESTART, 1)

    def clear_fault(self):
        """清除故障"""
        self.inst.write_register(self.REG_FAULTACK, 1)

    # ── 状态读取 ──────────────────────────────────

    @property
    def position(self) -> int:
        return self.inst.read_register(self.REG_CURPOS)

    @property
    def temperature(self) -> int:
        return self.inst.read_register(self.REG_CURTEMP)

    @property
    def current_ma(self) -> int:
        return self.inst.read_register(self.REG_CURCUR)

    @property
    def error_code(self) -> int:
        return self.inst.read_register(self.REG_CURERR)

    @property
    def force(self) -> int:
        return self.inst.read_register(self.REG_CURFORCE)

    @property
    def has_error(self) -> bool:
        return self.error_code != 0

    def status_dict(self) -> dict:
        """一键获取全部状态"""
        return {
            'position':    self.position,
            'temperature': self.temperature,
            'current_ma':  self.current_ma,
            'error_code':  self.error_code,
            'force':       self.force,
        }

    def close(self):
        self.inst.serial.close()


# ============================================================
# 使用示例
# ============================================================
if __name__ == '__main__':
    motor = ModbusMotor('/dev/ttyUSB0', slave_address=1)

    # 1. 读状态
    status = motor.status_dict()
    print(f"状态: {status}")

    # 2. 走到位置 1000 步（半行程）
    print("移动到 1000 步...")
    motor.move_to(1000)
    time.sleep(2)

    # 3. 确认到位
    print(f"当前位置: {motor.position} 步")

    # 4. 回到 0
    print("回到 0 步...")
    motor.move_to(0)

    motor.close()
```

---

## 9. 常用操作速查

### 定位控制

```python
motor.move_to(500)    # 走到 500 步（0=最短, 2000=最长）
motor.move_to(0)      # 缩回到最短位置
motor.move_to(2000)   # 伸出到最长位置
```

### 伺服模式（高频位置跟随）

```python
# ≥50Hz 循环写位置
while True:
    motor.move_to(target)
    time.sleep(0.018)    # ~55Hz
```

### 速度控制

```python
motor.set_speed(1000)   # 设速度为 1000 步/秒
motor.move_to(2000)     # 以该速度走到 2000
```

### 力控

```python
motor.set_force(500)    # 设力控目标为 500g（仅力传感器版支持）
```

### 紧急操作

```python
motor.emergency_stop()  # 急停！
motor.clear_fault()     # 清除故障
motor.restart()         # 恢复运行
```

### 状态监控

```python
print(motor.position)       # 当前位置
print(motor.current_ma)     # 电流
print(motor.temperature)    # 温度
print(motor.error_code)     # 故障码（0=正常）

if motor.has_error:
    print(f"故障！错误码: {motor.error_code}")
    motor.clear_fault()
```

---

## 10. 常见错误与排查

### 错误 1：`No response`

```
minimalmodbus.NoResponseError: No response
```

**原因与排查：**
- [ ] 电机上电了吗？（转换器的 24V 供电指示灯亮？）
- [ ] 接线对吗？（GND黑/VCC红/TX黄/RX蓝，颜色和 pin 脚对应关系确认）
- [ ] 波特率匹配吗？（电机出厂 921600，但**转换器 Modbus 侧默认是 9600**——你代码里波特率设对了吗？）
- [ ] **校验位设为 Even 了吗？**（最容易漏掉的参数！因时默认 Even，不是 None！）
- [ ] 从站地址对不对？（出厂都是 1，如果改过要确认）

### 错误 2：`CRC error` 或 `Checksum error`

- [ ] 通常是波特率或校验位不匹配
- [ ] 也可能是 RS485 总线有干扰 → 加 120Ω 终端电阻

### 错误 3：电机不动，但能读到寄存器值

- [ ] 电机是否在故障状态？读寄存器 0x0024，如果非 0，先 `clear_fault()`
- [ ] 写入的位置值是否在 0~2000 范围内？

### 错误 4：`ValueError: register address out of range`

- [ ] 寄存器地址写错了。因时的寄存器范围是 0x0001 ~ 0x0025
- [ ] 不要试图访问不存在的寄存器

### 错误 5：能控制一个电机，加第二个就不行

- [ ] 两个转换器的从站地址是否一样？（出厂默认都是 1！必须改成不同值）
- [ ] 改从站地址：写 0x0001 = 新地址，然后写 0x0003 = 1 保存，断电重启
- [ ] RS485 总线上是否只接了一个转换器时改了地址？（必须在独立连接时改，不能在总线上有多个相同地址设备时改）

### 调试技巧

```python
# 最简单的通信测试：读一个只读寄存器
import minimalmodbus

motor = minimalmodbus.Instrument('/dev/ttyUSB0', slaveaddress=1)
motor.serial.baudrate = 115200
motor.serial.bytesize = 8
motor.serial.parity = minimalmodbus.serial.PARITY_EVEN
motor.serial.stopbits = 1
motor.serial.timeout = 0.5

# 如果能成功读到值 → 通信 OK
# 如果报错 → 检查接线/供电/参数
try:
    pos = motor.read_register(0x0021)   # 读当前位置
    print(f"通信成功！当前电机位置: {pos}")
except Exception as e:
    print(f"通信失败: {e}")
```

---

## 11. 从单电机到 6 电机：多从站轮询

```
                          RS485 总线 (A/B/GND)
 ┌──────┐   ┌────────┬────────┬───┬────────┐
 │ PC   │   │   #1   │   #2   │   │   #6   │
 │USB-  │   │ addr=1 │ addr=2 │ … │ addr=6 │
 │RS485 │   │  →电机1│  →电机2│   │  →电机6│
 └──────┘   └────────┴────────┴───┴────────┘
  1 个串口              6 个从站共享同一根总线
```

```python
#!/usr/bin/env python3
"""6 电机 Modbus RTU 轮询控制"""

import time
import minimalmodbus


def create_motor(port: str, addr: int) -> minimalmodbus.Instrument:
    """创建 Modbus instrument"""
    m = minimalmodbus.Instrument(port, slaveaddress=addr)
    m.serial.baudrate = 115200
    m.serial.bytesize = 8
    m.serial.parity = minimalmodbus.serial.PARITY_EVEN
    m.serial.stopbits = 1
    m.serial.timeout = 0.1
    return m


# 6 个电机，共享同一个串口！用 slaveaddress 区分
MOTORS = {
    f'motor_{i}': create_motor('/dev/ttyUSB0', addr=i)
    for i in range(1, 7)   # 地址 1, 2, 3, 4, 5, 6
}


def move_all(positions: dict):
    """所有电机一起移动到各自的目标位置（逐个轮询发出）"""
    for name, pos in positions.items():
        try:
            MOTORS[name].write_register(0x0020, pos)
        except Exception as e:
            print(f"{name} 通信失败: {e}")


def read_all_positions() -> dict:
    """读取所有电机的当前位置"""
    results = {}
    for name, motor in MOTORS.items():
        try:
            results[name] = motor.read_register(0x0021)
        except Exception as e:
            results[name] = f"ERR: {e}"
    return results


def emergency_stop_all():
    """所有电机急停"""
    for name, motor in MOTORS.items():
        try:
            motor.write_register(0x0010, 1)
        except Exception:
            pass
    print("所有电机已急停！")


# ============================================================
# 使用示例
# ============================================================
if __name__ == '__main__':
    # 所有电机走到 1000 步
    move_all({
        'motor_1': 1000,
        'motor_2': 1000,
        'motor_3': 1000,
        'motor_4': 1000,
        'motor_5': 1000,
        'motor_6': 1000,
    })
    time.sleep(2)

    # 读回所有位置
    positions = read_all_positions()
    for name, pos in positions.items():
        print(f"  {name}: {pos} 步")

    # 清理
    for m in MOTORS.values():
        m.serial.close()
```

### 轮询频率估算（115200 bps 下）

```
单次写寄存器 (0x06) ≈ 3ms（含帧间隔和处理延迟）
6 电机轮询      ≈ 6 x 3ms = 18ms
有效更新频率    ≈ 1000/18 ≈ 55 Hz
```

**55 Hz ≥ 50 Hz 的伺服模式最低要求**，刚好够用。

> 如果需要更高频率或更低延迟，可以考虑分组：把 6 个电机分成 3 组，每组 2 个，用 3 路独立的 RS485 总线。每路只需 2 电机轮询 → 约 160 Hz。

---

## 附录：术语对照表

| 因时原生协议 | Modbus RTU | 说明 |
|-------------|------------|------|
| motor_id | slave address（从站地址） | 设备唯一标识 |
| 寄存器地址 (Register) | Register Address（寄存器地址） | 数据存储位置 |
| Checksum（校验和） | CRC16 | 帧校验方式 |
| 小端序（LE） | **大端序（BE）** | 多字节数据的字节序 |
| 波特率 921600 | 波特率 115200 | Modbus 侧的最高速率 |
| pack_xxx / unpack_xxx | write_register / read_register | 不用自己拼帧了 |
| serial.Serial | minimalmodbus.Instrument | 串口库 |

---

## 参考

- minimalmodbus 文档：https://minimalmodbus.readthedocs.io/
- Modbus RTU 协议规范：https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
- 因时 Modbus RTU 转换器说明书：PRJ-01-TS-U-011 V1.0.2
- 你的现有项目：https://github.com/LiorenYuhang/inspire_motor_control
