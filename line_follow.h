/**
 * @file    line_follow.h
 * @brief   8通道灰度传感器循迹算法 (PD控制器)
 *
 *          传感器阵列: 8个灰度传感器, 从左到右排列
 *          数字量: bit=0 表示检测到黑线, bit=1 表示白色地面
 *
 *          位置算法: 取最左和最右黑点的中点 (边缘法),
 *          相比平均值法能覆盖全量程, 不会因线宽稀释偏差。
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* ========================= PID 参数 (可在线调参) ========================= */
#define LF_KP           20.0f   /* 比例系数: 误差1个传感器 → 输出58 */
#define LF_KD           3.0f    /* 微分系数: 抑制震荡 */
#define LF_KI           0.0f    /* 积分系数: 0=关闭, 需要消除稳态误差时设为0.5~2 */

/* ========================= 速度参数 ========================= */
#define LF_BASE_SPEED   25      /* 基准速度 (%) */
#define LF_MIN_SPEED    0       /* 最低速度 (%) */
#define LF_MAX_SPEED    50     /* 最高速度 (%) */

/* ========================= 传感器参数 ========================= */
#define LF_SENSOR_COUNT 8       /* 传感器数量 */
#define LF_CENTER       3.5f    /* 目标中心 (0~7的中间) */

/* ========================= 函数声明 ========================= */

void LineFollow_Init(void);

void LineFollow_Run(uint8_t digital_sensors,
                    uint8_t *p_left_speed,
                    uint8_t *p_right_speed);

#endif /* LINE_FOLLOW_H */
