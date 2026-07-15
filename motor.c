#include "motor.h"

#include <stdbool.h>
#include <stddef.h>

#include "ti_msp_dl_config.h"

/* 机械参数和速度闭环参数。 */
#define MOTOR_ENCODER_PULSES_PER_REV  260.0f
#define MOTOR_WHEEL_DIAMETER_MM       47.0f
#define MOTOR_SPEED_SAMPLE_HZ         10.0f
#define MOTOR_PI                      3.14159f

#define MOTOR_PWM_MAX                 1000U
#define MOTOR_PWM_PER_MM_S            0.58f
#define MOTOR_CONTROL_DT_S            0.10f
#define MOTOR_KP                      0.25f
#define MOTOR_KI                      0.50f
#define MOTOR_INTEGRAL_LIMIT          400.0f

typedef struct {
    volatile int32_t pending_pulses; /* 本采样周期内累计的脉冲 */
    int32_t sampled_pulses;          /* 上一次采样得到的脉冲数 */
    float target_speed_mm_s;         /* 速度闭环目标 */
    float measured_speed_mm_s;       /* 编码器换算出的速度 */
    float integral;                  /* PI 控制器积分项 */
    uint16_t pwm;                    /* 当前 PWM 输出 */
} motor_state_t;

/* 所有电机内部状态只保存在本文件，其他模块不能直接修改。 */
static motor_state_t s_motors[MOTOR_COUNT];

/* 防止非法电机编号导致数组越界。 */
static bool motor_id_is_valid(motor_id_t motor)
{
    return (motor == MOTOR_LEFT) || (motor == MOTOR_RIGHT);
}

/* 将浮点数限制到指定范围。 */
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

/* 速度闭环当前只关心速度大小，因此取编码器脉冲绝对值。 */
static int32_t absolute_pulses(int32_t pulses)
{
    return (pulses < 0) ? -pulses : pulses;
}

/* 根据方向设置驱动芯片的两个方向引脚。 */
static void set_direction_pins(uint32_t in1_pin,
                               uint32_t in2_pin,
                               motor_direction_t direction)
{
    if (direction == MOTOR_STOP) {
        DL_GPIO_setPins(MOTOR_PORT, in1_pin | in2_pin);
    } else if (direction == MOTOR_FORWARD) {
        DL_GPIO_setPins(MOTOR_PORT, in1_pin);
        DL_GPIO_clearPins(MOTOR_PORT, in2_pin);
    } else {
        DL_GPIO_clearPins(MOTOR_PORT, in1_pin);
        DL_GPIO_setPins(MOTOR_PORT, in2_pin);
    }
}

/* 每 100 ms 读取并清空脉冲计数，再换算成 mm/s。 */
static void sample_wheel_speeds(void)
{
    motor_id_t motor;

    for (motor = MOTOR_LEFT; motor < MOTOR_COUNT; motor++) {
        int32_t pulses = s_motors[motor].pending_pulses;

        s_motors[motor].pending_pulses = 0;
        pulses = absolute_pulses(pulses);
        s_motors[motor].sampled_pulses = pulses;
        s_motors[motor].measured_speed_mm_s =
            ((float)pulses / MOTOR_ENCODER_PULSES_PER_REV) *
            MOTOR_PI * MOTOR_WHEEL_DIAMETER_MM * MOTOR_SPEED_SAMPLE_HZ;
    }
}

/* 前馈加 PI：前馈给出基础 PWM，PI 修正负载和左右轮差异。 */
static void update_speed_controller(motor_id_t motor)
{
    motor_state_t *state = &s_motors[motor];
    float error;
    float output;

    if (state->target_speed_mm_s <= 0.0f) {
        state->integral = 0.0f;
        motor_set_pwm(motor, 0U);
        return;
    }

    error = state->target_speed_mm_s - state->measured_speed_mm_s;
    state->integral += error * MOTOR_CONTROL_DT_S;
    state->integral = clamp_float(state->integral,
                                  -MOTOR_INTEGRAL_LIMIT,
                                  MOTOR_INTEGRAL_LIMIT);

    output = MOTOR_PWM_PER_MM_S * state->target_speed_mm_s;
    output += MOTOR_KP * error;
    output += MOTOR_KI * state->integral;
    output = clamp_float(output, 0.0f, (float)MOTOR_PWM_MAX);

    motor_set_pwm(motor, (uint16_t)output);
}

void motor_init(void)
{
    motor_id_t motor;

    for (motor = MOTOR_LEFT; motor < MOTOR_COUNT; motor++) {
        s_motors[motor] = (motor_state_t){0};
    }

    /* 拉高 STBY，使能电机驱动芯片。 */
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_SBYT_PIN);
    motor_set_direction(MOTOR_LEFT, MOTOR_STOP);
    motor_set_direction(MOTOR_RIGHT, MOTOR_STOP);
    motor_set_pwm(MOTOR_LEFT, 0U);
    motor_set_pwm(MOTOR_RIGHT, 0U);

    /* 启动 PWM 定时器和 100 ms 速度闭环定时器。 */
    DL_Timer_startCounter(PWM_MOTOR_INST);
    DL_Timer_startCounter(TIMER_0_INST);

    /* SysConfig 配置了引脚中断，这里再打开对应的 NVIC 中断。 */
    NVIC_ClearPendingIRQ(encoder_INT_IRQN);
    NVIC_EnableIRQ(encoder_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

void motor_set_direction(motor_id_t motor, motor_direction_t direction)
{
    if (!motor_id_is_valid(motor)) {
        return;
    }

    if (motor == MOTOR_LEFT) {
        set_direction_pins(MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, direction);
        return;
    }

    /* 右电机接线极性相反，因此在软件中交换前进和后退。 */
    if (direction == MOTOR_FORWARD) {
        direction = MOTOR_BACKWARD;
    } else if (direction == MOTOR_BACKWARD) {
        direction = MOTOR_FORWARD;
    }
    set_direction_pins(MOTOR_BIN1_PIN, MOTOR_BIN2_PIN, direction);
}

void motor_set_pwm(motor_id_t motor, uint16_t pwm)
{
    DL_TIMER_CC_INDEX channel;

    if (!motor_id_is_valid(motor)) {
        return;
    }

    if (pwm > MOTOR_PWM_MAX) {
        pwm = MOTOR_PWM_MAX;
    }

    channel = (motor == MOTOR_LEFT) ? GPIO_PWM_MOTOR_C0_IDX
                                    : GPIO_PWM_MOTOR_C1_IDX;
    s_motors[motor].pwm = pwm;
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, pwm, channel);
}

void motor_set_speed_target(motor_id_t motor, float speed_mm_s)
{
    if (!motor_id_is_valid(motor)) {
        return;
    }

    s_motors[motor].target_speed_mm_s =
        (speed_mm_s > 0.0f) ? speed_mm_s : 0.0f;
}

void motor_stop_all(void)
{
    motor_id_t motor;

    /* 修改多项控制状态时暂时关闭速度环中断，避免读到半更新数据。 */
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    for (motor = MOTOR_LEFT; motor < MOTOR_COUNT; motor++) {
        s_motors[motor].pending_pulses = 0;
        s_motors[motor].sampled_pulses = 0;
        s_motors[motor].target_speed_mm_s = 0.0f;
        s_motors[motor].measured_speed_mm_s = 0.0f;
        s_motors[motor].integral = 0.0f;
        motor_set_pwm(motor, 0U);
        motor_set_direction(motor, MOTOR_STOP);
    }
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

void motor_get_telemetry(motor_id_t motor, motor_telemetry_t *telemetry)
{
    if (!motor_id_is_valid(motor) || (telemetry == NULL)) {
        return;
    }

    telemetry->sampled_pulses = s_motors[motor].sampled_pulses;
    telemetry->target_speed_mm_s = s_motors[motor].target_speed_mm_s;
    telemetry->measured_speed_mm_s = s_motors[motor].measured_speed_mm_s;
    telemetry->pwm = s_motors[motor].pwm;
}

void GROUP1_IRQHandler(void)
{
    /* A 相上升沿触发中断，读取 B 相判断旋转方向。 */
    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
    case encoder_AA_IIDX: /* PA17/PA16：左轮编码器 */
        s_motors[MOTOR_LEFT].pending_pulses +=
            DL_GPIO_readPins(GPIOA, encoder_AB_PIN) ? 1 : -1;
        break;

    case encoder_BA_IIDX: /* PA15/PA14：右轮编码器 */
        s_motors[MOTOR_RIGHT].pending_pulses +=
            DL_GPIO_readPins(GPIOA, encoder_BB_PIN) ? 1 : -1;
        break;

    default:
        break;
    }
}

void TIMA0_IRQHandler(void)
{
    /* 每 100 ms 更新一次测速结果和左右轮 PI 输出。 */
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) != DL_TIMER_IIDX_LOAD) {
        return;
    }

    sample_wheel_speeds();
    update_speed_controller(MOTOR_LEFT);
    update_speed_controller(MOTOR_RIGHT);
}
