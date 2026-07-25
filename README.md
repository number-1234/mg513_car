# MSPM0G3507 循迹小车

程序框架按照参考工程的 `main + Control + Encoder + Sensor + GYRO + sys` 结构整理。路线状态使用全局 `flag` 和直接的 `if / else if` 判断，方便比赛时快速修改。

## 程序结构

```text
main.c                 主循环、flag、定时器中断、编码器中断
Control/control.c      电机方向、PWM、速度 PI、循迹和陀螺仪控制
Encoder/Encoder.c      编码器计数和左右轮测速
Sensor/Sensor.c        灰度读取、循迹权重、路线判断和黑白标定
GYRO/GYRO.c            MPU6050 DMP 初始化和角度读取
sys/sys.c              延时、串口 printf、限幅和角度处理
```

以下文件属于本车硬件底层，因传感器和陀螺仪型号与参考工程不同而保留：

```text
ADC.c / ADC.h
No_Mcu_Ganv_Grayscale_Sensor.c
No_Mcu_Ganv_Grayscale_Sensor_Config.h
mpu6050/
delay.h
empty.syscfg
```

`delay.h` 只给现有灰度和 MPU6050 驱动提供兼容声明，延时实现已经放入 `sys/sys.c`。

## flag 状态

| flag | 车辆动作 |
| --- | --- |
| `1` | 按 8 路灰度权重循迹 |
| `2` | 丢线后停车等待 1 秒 |
| `3` | 以当前 Yaw 为 0°，单轮转向 55° |
| `4` | MPU6050 保持方向直行，找到黑线后回到 flag 1 |

路线状态判断集中在 `Sensor/Sensor.c` 的 `Follow_Route()`，每个状态对应一个直接的 `if`。车辆动作集中在 `Control/control.c` 的 `Control()`。

## 常用修改位置

- 循迹速度、直行速度、转向角度、转向速度和 PID：`Control/control.c` 文件顶部。
- 丢线等待时间、传感器标定开关和黑白值：`Sensor/Sensor.c` 文件顶部。
- 路线状态切换：`Sensor/Sensor.c` 的 `Follow_Route()`。
- 主循环和中断调用顺序：`main.c`。

## 循迹权重

`Incremental_Quantity()` 使用八个直接的 `if`：

```text
-12  -9  -7  -3  +3  +7  +9  +12
```

`Sensor_Bits == 0xFF` 表示全白丢线；`0x00` 表示全黑，继续作为黑线或交叉区域处理。

## 硬件保持不变

本次结构调整没有修改 `empty.syscfg`：

- 左电机 PWM：PA21
- 左电机方向：PA26、PA25
- 左编码器：PA17、PA16
- 右电机 PWM：PA22
- 右电机方向：PA24、PA23
- 右编码器：PA15、PA14
- 电机驱动 STBY：PA2
- MPU6050 硬件 I²C：PA0、PA1
- 灰度 ADC：PA27

## 构建环境

- Code Composer Studio Debug
- TI Arm Clang 4.0.4 LTS
- MSPM0 SDK 2.10.0.04
- SysConfig 1.26.2
