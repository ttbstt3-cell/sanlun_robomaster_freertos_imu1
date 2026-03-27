# 三连轮战车控制系统

基于 **STM32F407** + **FreeRTOS** 的 RoboMaster 战车控制代码，支持 SBUS 遥控、4路舵机控制、BMI088 IMU 惯性测量以及 USB 虚拟串口数据回传。

## 硬件配置

| 项目 | 型号/参数 |
|------|-----------|
| 主控 | RoboMaster C板 (STM32F407IGHx) |
| 主频 | 168MHz (HSE PLL) |
| 遥控 | 乐迪遥控器 (SBUS 信号) |
| IMU | BMI088 (陀螺仪 + 加速度计) |
| 通信 | USB CDC 虚拟串口 / UART3 SBUS / SPI1 BMI088 |

## 架构

**FreeRTOS 3任务并发：**

| 任务 | 函数 | 优先级 | 周期 | 功能 |
|------|------|--------|------|------|
| LED心跳 | `StartLedTask` | Low | 500ms | 系统状态指示 |
| 底盘控制 | `StartControlTask` | High | 10ms | SBUS + 舵机控制 |
| IMU采集 | `StartIMUTask` | Realtime | 2ms | BMI088读取 + USB回传 |

## 功能特点

### 1. 双模式底盘控制

**模式1 - 驾驶模式 (SwB 拨杆在下)**
- 左摇杆上下 (CH3) → 转向角度
- 右摇杆上下 (CH2) → 整体驱动速度
  - 1号大轮：按 50/63 比例换算
  - 3号/4号小轮：直驱

**模式2 - 调试模式 (SwB 拨杆在上)**
- 右摇杆上下 (CH2) → 1号大轮独立控制
- 右摇杆左右 (CH1) → 小轮测试速度
- SwE (CH5) → 切换控制 3号 或 4号轮

### 2. 失控保护

信号丢失超过 50ms 时，所有舵机输出 TRIM 补偿值，防止车辆溜车。

### 3. BMI088 IMU

- 陀螺仪量程: ±2000°/s
- 加速度计量程: ±3g
- 通过 SPI1 通信 (PA4=CS_ACCEL, PB0=CS_GYRO)
- 通过 USB CDC 以 VOFA+ FireWater 格式回传数据

### 4. USB 数据回传

连接电脑后自动识别为虚拟串口，发送格式：
```
gx,gy,gz,ax,ay,az
```
数值已放大1000倍（如 `1500` 表示 `1.5`）

### 5. LED 状态指示

| LED | 引脚 | 功能 |
|-----|------|------|
| 蓝灯 | PH10 | 系统心跳闪烁 |
| 绿灯 | PH12 | SBUS信号有效常亮 |
| 红灯 | PH11 | 信号丢失亮起 |

## 通道映射

| CH | 遥控器输入 | 功能 |
|----|------------|------|
| CH1 | 右摇杆左右 | 调试模式控制小轮 |
| CH2 | 右摇杆上下 | 主驱动油门 / 1号轮 |
| CH3 | 左摇杆上下 | 转向控制 |
| CH5 | SwE | 选择3号/4号轮 |
| CH9 | SwB | 驾驶/调试模式切换 |

## 文件结构

```
sanlun_robomaster/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # 全局定义，片选引脚，TRIM宏
│   │   ├── sbus.h              # SBUS协议接口
│   │   ├── servo.h             # 舵机控制接口
│   │   ├── BMI088driver.h      # BMI088驱动接口
│   │   ├── BMI088Middleware.h   # 硬件抽象层接口
│   │   └── struct_typedef.h    # 基础类型定义
│   └── Src/
│       ├── main.c               # 入口，硬件初始化，任务创建
│       ├── freertos.c           # 3个RTOS任务实现
│       ├── sbus.c              # SBUS协议解析，中断回调
│       ├── servo.c              # 舵机PWM控制
│       ├── BMI088Middleware.c   # GPIO/SPI/Delay抽象
│       └── BMI088driver.c       # 大疆BMI088驱动
├── Drivers/                     # STM32 HAL库
├── Middlewares/                  # FreeRTOS + USB Device Library
├── USB_DEVICE/                  # USB CDC配置
├── MDK-ARM/                     # Keil工程文件
└── sanlun_robomaster.ioc       # CubeMX配置文件
```

## 编译

使用 **Keil MDK 5** 打开 `sanlun_robomaster.uvprojx`，编译并下载。

## TRIM 调参

360度舵机存在硬件公差，摇杆居中时轮子可能缓慢转动。通过 TRIM 值补偿：

| 参数 | 对应轮 | 轮子后退 → 增大 | 轮子前进 → 减小 |
|------|--------|-----------------|-----------------|
| TRIM_M1 | 1号大轮 | + | - |
| TRIM_M3 | 3号小轮 | + | - |
| TRIM_M4 | 4号小轮 | + | - |

修改位置: `Core/Src/freertos.c` 中的宏定义

## 调参步骤

1. 编译并烧录程序
2. 打开遥控器，确认绿灯亮（SBUS有效）
3. 不要触碰任何摇杆，观察哪个轮子在转动
4. 根据转动方向调整对应 TRIM 值
5. 重新烧录，直到三个轮子完全静止

## 联系方式

MIT License
