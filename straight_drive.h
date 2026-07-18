#ifndef STRAIGHT_DRIVE_H
#define STRAIGHT_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * MPU6050直行PID参数。
 * 先调KP，再根据需要少量增加KI和KD；PID输出单位作为左右轮速度修正量。
 */
#define STRAIGHT_KP               3.0f  /* 比例系数：角度偏差越大，修正越大 */
#define STRAIGHT_KI               0.0f  /* 积分系数：用于消除长期固定偏差 */
#define STRAIGHT_KD               0.0f  /* 微分系数：用于抑制快速摆动 */
#define STRAIGHT_PID_LIMIT       30.0f  /* 修正量限幅，防止一侧轮子接近停转 */
#define STRAIGHT_PID_DT_S         0.01f /* DMP数据周期，100Hz对应0.01秒 */

/* 初始化MPU6050和DMP；返回0表示成功，非0表示失败步骤。 */
uint8_t straight_drive_init(void);

/*
 * 只更新MPU角度，不控制电机。
 * 循迹期间也应周期调用，以清空DMP FIFO并保存最新yaw。
 */
void straight_drive_update_sensor(void);

/* 开始一次新的直行过程，并准备把下一帧有效yaw记录为0度。 */
void straight_drive_start(void);

/*
 * 按给定基础速度直行，单位mm/s。
 * 如果还没调用start()，本函数会自动开始并记录新的角度零点。
 */
void straight_drive_run(float base_speed_mm_s);

/* 退出直行并清除PID状态；下次运行时会重新记录角度零点。 */
void straight_drive_reset(void);

/* 以下为简单状态读取接口，不使用任何遥测结构体。 */
bool straight_drive_is_ready(void);       /* MPU6050/DMP是否初始化成功 */
bool straight_drive_is_active(void);      /* 当前是否处于直行控制状态 */
float straight_drive_get_angle(void);     /* 相对本次零点的角度，单位度 */
float straight_drive_get_pid_output(void);/* 当前PID速度修正量 */
float straight_drive_get_yaw(void);       /* MPU6050输出的原始yaw角 */

/*
 * 以reference_yaw为0度，使用单轮转动到target_angle_deg。
 * 按当前实车方向，target_angle_deg为正时左轮转、右轮停；为负时相反。
 * 本函数需要在主循环中反复调用；到达目标角度并停车后返回true。
 */
bool straight_drive_turn_to_angle(float reference_yaw,
                                  float target_angle_deg,
                                  float turn_speed_mm_s);

#endif
