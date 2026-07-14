/**
 * @file    line_follow.c
 * @brief   循迹PD控制器实现
 *
 *          位置算法: 边缘法
 *            - 找到最左和最右检测到黑线的传感器
 *            - 位置 = (最左 + 最右) / 2
 *            - 优点: 无论线宽, 偏差始终覆盖全量程 (-3.5 ~ +3.5)
 *
 *          输出:
 *            左轮 = 基准速度 + PD输出
 *            右轮 = 基准速度 - PD输出
 */

#include "line_follow.h"
#include "motor.h"

/* ========================= 内部状态 ========================= */
static float g_lastError = 0.0f;
static float g_integral  = 0.0f;

/* ========================= 工具函数 ========================= */

static inline int clamp_i(int val, int min, int max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/* ========================= 位置计算 (边缘法) ========================= */

/**
 * @brief 用边缘法计算黑线中心位置
 * @param digital 8位数字量, bit=0表示黑线
 * @return 黑线中心位置 (0.0~7.0), -1.0表示完全没有黑线
 *
 * 示例:
 *   黑线在传感器 2,3,4 → left=2, right=4 → 位置=3.0
 *   黑线在传感器 0    → left=0, right=0 → 位置=0.0 (最左)
 *   黑线在传感器 7    → left=7, right=7 → 位置=7.0 (最右)
 */
static float CalcLinePosition(uint8_t digital)
{
    int left  = -1;   /* 最左黑传感器索引 */
    int right = -1;   /* 最右黑传感器索引 */

    for (int i = 0; i < LF_SENSOR_COUNT; i++) {
        if (!(digital & (1 << i))) {   /* bit=0 → 黑线 */
            if (left  < 0) left  = i;
            if (right < 0) right = i;
            right = i;   /* 持续更新最右 */
        }
    }

    if (left < 0) {
        return -1.0f;   /* 无黑线 */
    }

    return (float)(left + right) * 0.5f;
}

/* ========================= 公共函数 ========================= */

void LineFollow_Init(void)
{
    g_lastError = 0.0f;
    g_integral  = 0.0f;
}

void LineFollow_Run(uint8_t digital_sensors,
                    uint8_t *p_left_speed,
                    uint8_t *p_right_speed)
{
    float position, error, output;

    /* 1. 计算黑线位置 */
    position = CalcLinePosition(digital_sensors);

    if (position < 0.0f) {
        /* 没检测到线 → 保持直行 */
        *p_left_speed  = LF_BASE_SPEED;
        *p_right_speed = LF_BASE_SPEED;
        return;
    }

    /* 2. 偏差: 正=偏右, 负=偏左 */
    error = position - LF_CENTER;

    /* 3. PD计算 */
    output  = LF_KP * error;
    output += LF_KD * (error - g_lastError);
    output += LF_KI * g_integral;

    g_integral += error;
    g_lastError = error;

    /* 4. 速度分配 */
    int left  = (int)LF_BASE_SPEED + (int)output;
    int right = (int)LF_BASE_SPEED - (int)output;

    *p_left_speed  = (uint8_t)clamp_i(left,  LF_MIN_SPEED, LF_MAX_SPEED);
    *p_right_speed = (uint8_t)clamp_i(right, LF_MIN_SPEED, LF_MAX_SPEED);
}
