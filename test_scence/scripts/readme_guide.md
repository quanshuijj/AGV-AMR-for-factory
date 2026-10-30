# 车辆运动学验证系统

本系统用于验证车辆CAN控制和运动学模型的正确性，通过从虚拟CAN口vcan0读取数据进行实时验证。

## 系统组成

1. **vehicle_kinematics_validator.py** - 主验证器，从vcan0读取CAN数据并验证运动学模型
2. **can_test_sender.py** - CAN测试数据发送器，用于发送测试数据
3. **quick_start.sh** - 快速启动脚本，自动设置环境

## 快速开始

### 1. 环境设置

```bash
# 给启动脚本执行权限
chmod +x quick_start.sh

# 运行启动脚本 (会自动设置虚拟CAN接口)
./quick_start.sh
```

如果需要手动设置虚拟CAN接口：

```bash
# 加载vcan模块
sudo modprobe vcan

# 创建虚拟CAN接口
sudo ip link add dev vcan0 type vcan

# 启动接口
sudo ip link set up vcan0
```

### 2. 安装依赖

```bash
# 安装Python依赖
pip3 install python-can rospy geometry_msgs std_msgs gazebo_msgs tf

# 安装CAN工具 (可选，用于调试)
sudo apt-get install can-utils
```

### 3. 运行验证

#### 方式一：使用测试发送器验证

在终端1中运行验证器：
```bash
python3 vehicle_kinematics_validator.py
```

在终端2中运行测试发送器：
```bash
# 运行完整测试套件
python3 can_test_sender.py --mode test

# 交互模式 (手动输入参数)
python3 can_test_sender.py --mode interactive

# 连续发送特定参数
python3 can_test_sender.py --mode continuous --vx 1.0 --rz 0.1 --control 0
```

#### 方式二：运行你的C++程序

确保你的C++程序配置为向vcan0发送CAN消息，然后：

1. 运行验证器：
```bash
python3 vehicle_kinematics_validator.py
```

2. 运行你的C++程序，它会自动发送CAN数据到vcan0

### 4. 监控CAN消息 (可选)

如果安装了can-utils，可以监控CAN消息：

```bash
# 监控所有CAN消息
candump vcan0

# 只监控特定ID的消息
candump vcan0,8F110D8:FFFFFFFF  # 转向控制
candump vcan0,8F210D8:FFFFFFFF  # 电机控制
```

## CAN协议说明

### 转向控制消息 (ID: 0x8F110D8)
| 字节 | 含义 | 格式 |
|------|------|------|
| 0 | 右前轮转向角 | 0.2°/bit，偏置128 |
| 1 | 左前轮转向角 | 0.2°/bit，偏置128 |
| 2 | 右后轮转向角 | 0.2°/bit，偏置128 |
| 3 | 左后轮转向角 | 0.2°/bit，偏置128 |
| 4 | 控制模式 | 0-阿克曼 1-横/斜行 3-遥控 |

### 电机控制消息 (ID: 0x8F210D8)
| 字节 | 含义 | 格式 |
|------|------|------|
| 0 | 左前电机1 | 12rpm/bit，偏置128 |
| 1 | 左前电机2 | 12rpm/bit，偏置128 |
| 2 | 右前电机1 | 12rpm/bit，偏置128 |
| 3 | 右前电机2 | 12rpm/bit，偏置128 |
| 4 | 左后电机1 | 12rpm/bit，偏置128 |
| 5 | 左后电机2 | 12rpm/bit，偏置128 |
| 6 | 右后电机1 | 12rpm/bit，偏置128 |
| 7 | 右后电机2 | 12rpm/bit，偏置128 |

## 车辆参数

验证系统使用以下车辆参数（与C++代码保持一致）：

- **轴距 (L)**: 2.85m
- **轮距 (W)**: 1.64m  
- **双轮间距 (W2)**: 0.42m
- **轮胎周长**: 1.885m
- **最小转弯半径**: 6.0m

## 控制模式

- **0**: 四轮转向 (四轮阿克曼)
- **1**: 前轮转向 (两轮阿克曼)
- **2**: 原地旋转 (未实现)
- **3**: 遥控模式

## 验证功能

验证器会自动检查以下内容：

### 1. 转向角度验证
- 阿克曼转向几何一致性
- 转向角度范围检查 (±24°)
- 转弯半径合理性

### 2. 轮速验证
- 各轮速度一致性
- 差速比例正确性
- 运动学约束满足

### 3. CAN数据解析
- 消息格式正确性
- 数据范围有效性
- 时间同步检查

## 输出结果

验证器会输出详细的验证结果：

```
=============================================================
运动学验证结果: PASS
=============================================================
估算运动: vx=1.000m/s, rz=0.100rad/s
控制模式: 0
转弯半径: 10.00m
最大转向角误差: 0.15°
最大轮速误差: 0.023m/s
=============================================================
```

### 验证状态说明
- **PASS**: 验证通过，误差在可接受范围内
- **WARNING**: 有警告，误差较大但可接受
- **FAIL**: 验证失败，存在严重错误

## ROS集成 (可选)

如果安装了ROS，验证器会发布以下话题：

- `/cmd_vel` - 估算的车辆运动速度
- `/validation_result` - 详细验证结果 (JSON格式)
- `/decoded_steering` - 解码后的转向数据
- `/decoded_motor` - 解码后的电机数据

## Gazebo集成 (可选)

如果Gazebo服务可用，验证器会自动发布运动命令到Gazebo进行可视化。

## 故障排除

### 1. CAN接口问题

**错误**: `无法连接到CAN接口 vcan0`

**解决方案**:
```bash
# 检查接口是否存在
ip link show vcan0

# 重新创建接口
sudo ip link delete vcan0  # 如果已存在
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 2. 权限问题

**错误**: `Permission denied`

**解决方案**:
```bash
# 将用户添加到dialout组
sudo usermod -a -G dialout $USER

# 重新登录或运行
newgrp dialout
```

### 3. Python依赖问题

**错误**: `ModuleNotFoundError: No module named 'can'`

**解决方案**:
```bash
# 安装missing包
pip3 install python-can

# 如果使用conda
conda install python-can
```

### 4. 无CAN消息接收

**问题**: 验证器运行但收不到CAN消息

**检查步骤**:
1. 确认vcan0接口状态: `ip link show vcan0`
2. 检查是否有程序在发送: `candump vcan0`
3. 确认CAN ID正确: `0x8F110D8`, `0x8F210D8`

## 测试场景

系统预设了以下测试场景：

1. **直行测试** - vx=1.0m/s, rz=0rad/s
2. **左转测试** - vx=1.0m/s, rz=0.1rad/s  
3. **右转测试** - vx=1.0m/s, rz=-0.1rad/s
4. **低速转弯** - vx=0.5m/s, rz=0.2rad/s
5. **高速转弯** - vx=2.0m/s, rz=0.05rad/s
6. **倒车测试** - vx=-1.0m/s, rz=0.1rad/s
7. **不同控制模式测试**

## 自定义测试

可以通过以下方式添加自定义测试：

### 1. 修改测试发送器

编辑`can_test_sender.py`中的`run_test_suite()`函数，添加新的测试用例：

```python
test_cases = [
    TestCase("自定义测试", 1.5, 0.15, 0, 5.0),
    # 添加更多测试用例...
]
```

### 2. 使用交互模式

运行交互模式手动输入参数：
```bash
python3 can_test_sender.py --mode interactive
```

### 3. 直接调用C++程序

确保你的C++程序能接受命令行参数或配置文件来设置不同的运动参数。

## 性能调优

### 1. 消息频率调整

默认CAN消息发送频率为50ms (20Hz)，可在代码中调整：

```python
time.sleep(0.05)  # 修改此值调整频率
```

### 2. 验证精度调整

在验证器中调整误差阈值：

```python
if max_angle_error < 2.0 and max_speed_error < 0.1:  # 调整阈值
    result['validation']['status'] = 'PASS'
```

## 扩展功能

### 1. 数据记录

可以添加数据记录功能，保存验证结果到文件：

```python
import csv
import datetime

# 在验证结果中添加时间戳和保存逻辑
```

### 2. 实时可视化

可以集成matplotlib或其他工具实现实时数据可视化：

```python
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 添加实时绘图功能
```

### 3. Web界面

可以添加Web界面进行远程监控和控制。

## 联系信息

如有问题或建议，请联系开发团队。

---

**注意**: 本验证系统仅用于开发和测试目的，实际车辆控制请确保安全性验证。