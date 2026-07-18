#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* 循迹控制器参数。
 * 速度值表示满量程速度的百分比，最终由 car_control.c 换算成 mm/s。 */
#define LF_KP                  4.0f
#define LF_KD                  0.5f
#define LF_KI                  0.0f
#define LF_BASE_SPEED         30
#define LF_MIN_SPEED           0
#define LF_MAX_SPEED           35

/* 8 路传感器从左到右编号为 0～7，中心位置为 3.5。 */
#define LF_SENSOR_COUNT        8
#define LF_CENTER              3.5f

/* 丢线后沿最后偏差方向搜索约 300 ms，然后停车。 */
#define LF_LOST_SEARCH_SPEED   25
#define LF_LOST_SEARCH_CYCLES  60
#define LF_LOST_MIN_ERROR      0.25f

/* 清空历史误差和丢线计数。 */
void LineFollow_Init(void);

/* 根据 8 路数字量计算左右轮速度百分比。
 * digital_sensors 中位为 0 表示检测到黑线。 */
void LineFollow_Run(uint8_t digital_sensors,
                    uint8_t *p_left_speed,
                    uint8_t *p_right_speed);

#endif
