#ifndef SYS_SCREEN_H
#define SYS_SCREEN_H

#include <stdint.h>

/* 三易串口屏 — 把数据推送到屏幕控件 */
void Screen_Update(int flag, uint8_t sensor, float yaw,
                   float l_vel, float r_vel,
                   int l_pwm, int r_pwm);

#endif
