# MSPM0G3507 循迹小车

这是一个基于 TI MSPM0G3507、DriverLib 和 8 路灰度传感器的双电机循迹车工程。软件分为应用配置、车辆控制、电机闭环、传感器校准和基础硬件接口几个层次。

## 当前运行模式

所有应用开关都集中在 `app_config.h`：

- `APP_SENSOR_CALIBRATION_ENABLED`：设为 `1` 时运行黑白校准。
- `APP_MOTOR_TEST_MODE`：设为 `1` 时绕过循迹，只测试左右轮速度闭环。
- `APP_MOTOR_TEST_TARGET_MM_S`：电机测试目标速度，当前为 500 mm/s。
- `APP_TELEMETRY_PERIOD_MS`：串口遥测周期，当前为 100 ms。

当前工程处于电机闭环测试模式，左右轮目标速度均为 500 mm/s。

## 模块职责

| 模块 | 职责 |
| --- | --- |
| `main.c` | 系统启动、模式选择、主循环和串口遥测 |
| `app_config.h` | 集中管理应用模式和运行参数 |
| `car_control.c/.h` | 组合传感器、循迹输出和左右电机目标速度 |
| `motor.c/.h` | 电机方向/PWM、编码器采样、100 ms PI 速度闭环和遥测 |
| `calibrate.c/.h` | 灰度传感器黑白校准流程 |
| `ADC.c/.h` | 阻塞式 ADC 采样接口 |
| `Uart.c/.h` | UART 字符和字符串发送接口 |
| `delay.c/.h` | SysTick 毫秒时基和微秒/毫秒延时 |
| `line_follow.c/.h` | 循迹算法；本次重构未修改 |
| `No_Mcu_Ganv_Grayscale_Sensor.*` | 第三方灰度传感器驱动；本次重构未修改 |

## 电机闭环

- 左电机 PWM：PA21 / TIMG6 CCP0
- 左电机方向：PA26、PA25
- 左编码器：PA17、PA16
- 右电机 PWM：PA22 / TIMG6 CCP1
- 右电机方向：PA24、PA23
- 右编码器：PA15、PA14
- 电机驱动 STBY：PA2
- PWM 范围：0～1000
- 速度采样和 PI 更新周期：100 ms

电机状态由 `motor.c` 内部管理。其他模块只能通过公开函数设置方向、PWM、目标速度或读取 `motor_telemetry_t`，不会直接修改控制器内部变量。

## 串口遥测

输出示例：

```text
Pulse:89,88 Target:500,500 Speed:505,500 PWM:294,276
```

字段依次表示最近 100 ms 的编码器脉冲、目标速度、实测速度和 PWM。

## 灰度校准

1. 将 `APP_SENSOR_CALIBRATION_ENABLED` 设为 `1`。
2. 烧录后按串口提示依次放置在白色和黑色表面。
3. 将输出的校准数组更新到 `car_control.c`。
4. 将校准开关恢复为 `0`，重新构建。

## 构建环境

- Code Composer Studio Debug 配置
- TI Arm Clang 4.0.4 LTS
- MSPM0 SDK 2.10.0.04
- SysConfig 1.26.2
