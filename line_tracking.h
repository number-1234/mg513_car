#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 循迹功能对外接口。
 * 使用者不需要接触灰度驱动内部的No_MCU_Sensor结构体。
 */

/* 初始化灰度传感器、ADC中断和原来的LineFollow循迹算法。 */
void line_tracking_init(void);

/*
 * 读取一次灰度传感器并运行循迹算法。
 * 返回true：检测到线，函数内部已经设置左右轮目标速度。
 * 返回false：没有检测到线，调用者应切换到其他控制方式。
 */
bool line_tracking_run(void);

/* 返回最近一次line_tracking_run()是否检测到线。 */
bool line_tracking_has_line(void);

/* 返回8路传感器数字量，每一位对应一路灰度探头。 */
uint8_t line_tracking_get_sensor_bits(void);

/* 返回循迹算法最近计算出的左右轮速度百分比，范围0~100。 */
uint8_t line_tracking_get_left_percent(void);
uint8_t line_tracking_get_right_percent(void);

/* 返回从“有线”切换到“丢线”的累计次数。 */
uint16_t line_tracking_get_lost_count(void);

#endif
