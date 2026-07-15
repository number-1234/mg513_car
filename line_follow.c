#include "line_follow.h"

/* 上一次偏差用于微分项，也用于丢线时判断搜索方向。 */
static float s_last_error;
static float s_integral;
static uint16_t s_lost_cycles;

/* 将整数限制在指定范围内。 */
static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/* 使用检测到黑线的传感器平均索引计算黑线位置。
 * 返回范围为 0.0～7.0；完全没有检测到黑线时返回 -1.0。 */
static float calculate_line_position(uint8_t sensor_bits)
{
    float index_sum = 0.0f;
    uint8_t active_count = 0U;
    uint8_t index;

    for (index = 0U; index < LF_SENSOR_COUNT; index++) {
        if ((sensor_bits & (uint8_t)(1U << index)) == 0U) {
            index_sum += (float)index;
            active_count++;
        }
    }

    if (active_count == 0U) {
        return -1.0f;
    }
    return index_sum / (float)active_count;
}

/* 丢线时先朝最后看到黑线的方向单轮搜索，超时后双轮停止。 */
static void handle_lost_line(uint8_t *left_speed, uint8_t *right_speed)
{
    s_integral = 0.0f;
    if (s_lost_cycles < LF_LOST_SEARCH_CYCLES) {
        s_lost_cycles++;
    }

    if ((s_lost_cycles < LF_LOST_SEARCH_CYCLES) &&
        (s_last_error > LF_LOST_MIN_ERROR)) {
        *left_speed = LF_LOST_SEARCH_SPEED;
        *right_speed = 0U;
    } else if ((s_lost_cycles < LF_LOST_SEARCH_CYCLES) &&
               (s_last_error < -LF_LOST_MIN_ERROR)) {
        *left_speed = 0U;
        *right_speed = LF_LOST_SEARCH_SPEED;
    } else {
        *left_speed = 0U;
        *right_speed = 0U;
    }
}

void LineFollow_Init(void)
{
    s_last_error = 0.0f;
    s_integral = 0.0f;
    s_lost_cycles = 0U;
}

void LineFollow_Run(uint8_t digital_sensors,
                    uint8_t *p_left_speed,
                    uint8_t *p_right_speed)
{
    float position;
    float error;
    float correction;
    int left_speed;
    int right_speed;

    position = calculate_line_position(digital_sensors);
    if (position < 0.0f) {
        handle_lost_line(p_left_speed, p_right_speed);
        return;
    }

    /* 重新检测到黑线后清空丢线计数。正偏差表示黑线位于右侧。 */
    s_lost_cycles = 0U;
    error = position - LF_CENTER;

    /* 积分当前关闭，但保留实现便于以后消除稳定偏差。 */
    s_integral += error;
    if (s_integral > 10.0f) {
        s_integral = 10.0f;
    } else if (s_integral < -10.0f) {
        s_integral = -10.0f;
    }

    correction = LF_KP * error;
    correction += LF_KD * (error - s_last_error);
    correction += LF_KI * s_integral;
    s_last_error = error;

    /* 黑线偏右时左轮加速、右轮减速，使车头向右修正。 */
    left_speed = LF_BASE_SPEED + (int)correction;
    right_speed = LF_BASE_SPEED - (int)correction;

    *p_left_speed = (uint8_t)clamp_int(
        left_speed, LF_MIN_SPEED, LF_MAX_SPEED);
    *p_right_speed = (uint8_t)clamp_int(
        right_speed, LF_MIN_SPEED, LF_MAX_SPEED);
}
