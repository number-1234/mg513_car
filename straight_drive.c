#include "straight_drive.h"

#include "motor.h"
#include "mpu6050/inc/inv_mpu.h"

/* 单轮转向到达目标角度时允许的误差，单位为度。 */
#define TURN_ANGLE_TOLERANCE_DEG  2.0f

/*
 * 本文件只负责“MPU6050角度 -> 航向PID -> 左右轮差速”。
 * MPU6050和DMP底层驱动仍由mpu6050文件夹提供，本文件只调用公开接口。
 */

/* 运行状态标志。 */
static bool s_mpu_ready;      /* true表示MPU6050和DMP初始化成功 */
static bool s_angle_valid;    /* true表示至少成功读取过一次角度 */
static bool s_new_data;       /* true表示当前角度还没有参与PID计算 */
static bool s_active;         /* true表示当前正在执行直行控制 */
static bool s_zero_captured;  /* true表示本次直行已经记录了角度零点 */

/* DMP输出角度和本次直行使用的相对角度。 */
static float s_pitch;          /* 俯仰角，本控制暂时不使用 */
static float s_roll;           /* 横滚角，本控制暂时不使用 */
static float s_yaw;            /* 当前航向角，范围约-180~180度 */
static float s_zero_yaw;       /* 刚进入丢线直行时保存的yaw */
static float s_relative_angle; /* 当前yaw相对s_zero_yaw的变化量 */

/* PID状态单独保存；这里不使用结构体，方便观察和修改。 */
static float s_pid_error;       /* 当前角度误差：目标0度-相对角度 */
static float s_pid_last_error;  /* 上一次误差，用于微分计算 */
static float s_pid_integral;    /* 误差积分累计值 */
static float s_pid_derivative;  /* 误差变化速度 */
static float s_pid_output;      /* 最终左右轮速度修正量 */

/* 把数值限制在minimum和maximum之间，防止控制输出过大。 */
static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/*
 * 处理yaw跨越-180/180度的问题。
 * 例如179度转到-179度，实际只变化2度，而不是变化-358度。
 */
static float normalize_angle(float angle)
{
    if (angle > 180.0f) {
        angle -= 360.0f;
    } else if (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/* 清空PID历史数据，避免上一次直行影响下一次直行。 */
static void reset_pid(void)
{
    s_pid_error = 0.0f;
    s_pid_last_error = 0.0f;
    s_pid_integral = 0.0f;
    s_pid_derivative = 0.0f;
    s_pid_output = 0.0f;
}

/* 使用最新一帧DMP数据计算一次航向PID。 */
static float calculate_pid(void)
{
    /* DMP没有产生新数据时保持上一次输出，不重复积分或微分。 */
    if (!s_new_data) {
        return s_pid_output;
    }
    s_new_data = false;

    /* 本次直行的第一帧有效数据就是0度方向。 */
    if (!s_zero_captured) {
        s_zero_yaw = s_yaw;
        s_relative_angle = 0.0f;
        s_zero_captured = true;
        reset_pid();
        return 0.0f;
    }

    /* 目标相对角度固定为0度，所以error = 0 - relative_angle。 */
    s_relative_angle = normalize_angle(s_yaw - s_zero_yaw);
    s_pid_error = -s_relative_angle;

    /* 积分项按时间累计，并限制范围以防积分饱和。 */
    s_pid_integral += s_pid_error * STRAIGHT_PID_DT_S;
    s_pid_integral = clamp_float(s_pid_integral, -100.0f, 100.0f);

    /* 微分项表示相邻两帧误差变化速度。 */
    s_pid_derivative =
        (s_pid_error - s_pid_last_error) / STRAIGHT_PID_DT_S;
    s_pid_last_error = s_pid_error;

    /* P、I、D三项求和，再限制最大差速修正量。 */
    s_pid_output = STRAIGHT_KP * s_pid_error;
    s_pid_output += STRAIGHT_KI * s_pid_integral;
    s_pid_output += STRAIGHT_KD * s_pid_derivative;
    s_pid_output = clamp_float(s_pid_output,
                               -STRAIGHT_PID_LIMIT,
                               STRAIGHT_PID_LIMIT);
    return s_pid_output;
}

uint8_t straight_drive_init(void)
{
    uint8_t status;

    /* 先把本模块所有状态恢复成确定的初始值。 */
    s_mpu_ready = false;
    s_angle_valid = false;
    s_new_data = false;
    s_active = false;
    s_zero_captured = false;
    s_pitch = 0.0f;
    s_roll = 0.0f;
    s_yaw = 0.0f;
    s_zero_yaw = 0.0f;
    s_relative_angle = 0.0f;
    reset_pid();

    /* DMP只在这里初始化一次，主循环不能重复初始化。 */
    status = mpu_dmp_init();
    s_mpu_ready = (status == 0U);
    return status;
}

void straight_drive_update_sensor(void)
{
    /* 初始化失败时不访问DMP，直行会自动退化成左右轮同速。 */
    if (!s_mpu_ready) {
        return;
    }

    /* 返回0才说明FIFO中取到了一帧完整有效的姿态数据。 */
    if (mpu_dmp_get_data(&s_pitch, &s_roll, &s_yaw) == 0U) {
        s_angle_valid = true;
        s_new_data = true;
    }
}

void straight_drive_start(void)
{
    /*
     * 这里只把零点标记为“尚未记录”。
     * 下一次calculate_pid()收到有效yaw后才真正保存零点。
     */
    s_active = true;
    s_zero_captured = false;
    s_relative_angle = 0.0f;
    reset_pid();
}

void straight_drive_run(float base_speed_mm_s)
{
    float correction;

    /* 允许用户只调用run()，不要求必须提前调用start()。 */
    if (!s_active) {
        straight_drive_start();
    }

    /* 每个控制周期尝试读取一帧新角度。 */
    straight_drive_update_sensor();

    /* 没有有效角度时修正量为0，两侧先保持相同目标速度。 */
    if (s_angle_valid) {
        correction = calculate_pid();
    } else {
        correction = 0.0f;
    }

    /*
     * PID正输出时左轮加速、右轮减速；负输出时相反。
     * 这种符号方向已经与你当前小车的安装方向匹配。
     */
    motor_drive_forward(base_speed_mm_s + correction,
                        base_speed_mm_s - correction);
}

void straight_drive_reset(void)
{
    /* 不关闭DMP，只退出本次直行并等待下一次丢线重新记录零点。 */
    s_active = false;
    s_zero_captured = false;
    s_relative_angle = 0.0f;
    reset_pid();
}

/* 以下getter只读取内部状态，不执行传感器采样或电机控制。 */
bool straight_drive_is_ready(void)
{
    return s_mpu_ready;
}

bool straight_drive_is_active(void)
{
    return s_active;
}

float straight_drive_get_angle(void)
{
    return s_relative_angle;
}

float straight_drive_get_pid_output(void)
{
    return s_pid_output;
}

float straight_drive_get_yaw(void)
{
    return s_yaw;
}

bool straight_drive_turn_to_angle(float reference_yaw,
                                  float target_angle_deg,
                                  float turn_speed_mm_s)
{
    float current_angle;
    float angle_error;

    /* 获取最新yaw；没有有效姿态数据时保持停车，防止盲目旋转。 */
    straight_drive_update_sensor();
    if (!s_angle_valid) {
        motor_stop_all();
        return false;
    }

    /* 当前角度和参考零点做差，并处理yaw跨越-180/180度的问题。 */
    current_angle = normalize_angle(s_yaw - reference_yaw);
    /* 保存单轮转向的相对角度，供串口打印和调试使用。 */
    s_relative_angle = current_angle;
    angle_error = normalize_angle(target_angle_deg - current_angle);

    /* 进入目标角度允许范围后停车，true表示本次转向完成。 */
    if ((angle_error >= -TURN_ANGLE_TOLERANCE_DEG) &&
        (angle_error <= TURN_ANGLE_TOLERANCE_DEG)) {
        motor_stop_all();
        return true;
    }

    /* 防止传入负速度；方向只由角度误差的正负决定。 */
    if (turn_speed_mm_s < 0.0f) {
        turn_speed_mm_s = -turn_speed_mm_s;
    }

    if (angle_error > 0.0f) {
        /* 当前实车方向：左轮向前转、右轮停止时，yaw增大。 */
        motor_drive_forward(turn_speed_mm_s, 0.0f);
    } else {
        /* 当前实车方向：右轮向前转、左轮停止时，yaw减小。 */
        motor_drive_forward(0.0f, turn_speed_mm_s);
    }

    return false;
}
