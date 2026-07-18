#include "motor.h"

#include <stdbool.h>

#include "ti_msp_dl_config.h"

/*
 * 电机模块负责：方向控制、PWM输出、编码器测速和左右轮独立速度PI。
 * 对外只提供普通函数，不使用遥测结构体。
 */

/* 编码器和车轮机械参数，用于把100ms内脉冲数换算成mm/s。 */
#define MOTOR_ENCODER_PULSES_PER_REV  260.0f /* 车轮转一圈对应的编码器脉冲 */
#define MOTOR_WHEEL_DIAMETER_MM       47.0f  /* 车轮直径，单位mm */
#define MOTOR_SPEED_SAMPLE_HZ         10.0f  /* 100ms测速一次，即每秒10次 */
#define MOTOR_PI                      3.14159f

/* PWM和电机速度PI参数。 */
#define MOTOR_PWM_MAX                 1000U  /* PWM计数最大值 */
#define MOTOR_PWM_PER_MM_S            0.58f  /* 速度转基础PWM的前馈系数 */
#define MOTOR_CONTROL_DT_S            0.10f  /* 速度环周期，单位秒 */
#define MOTOR_KP                      0.25f  /* 速度比例系数 */
#define MOTOR_KI                      0.50f  /* 速度积分系数 */
#define MOTOR_INTEGRAL_LIMIT          400.0f /* 速度积分限幅 */

/*
 * 左右轮状态分别放在普通数组中：下标0是左轮，下标1是右轮。
 * volatile表示这些变量会在主程序和中断之间共享。
 */
static volatile int32_t s_pending_pulses[MOTOR_COUNT]; /* 本周期编码器脉冲 */
static volatile float s_target_speed[MOTOR_COUNT];     /* 目标速度mm/s */
static volatile float s_measured_speed[MOTOR_COUNT];   /* 实测速度mm/s */
static float s_speed_integral[MOTOR_COUNT];            /* 速度PI积分项 */
static volatile uint16_t s_pwm[MOTOR_COUNT];           /* 当前PWM值 */

/* 检查传入的电机编号是否为左轮或右轮，防止数组越界。 */
static bool motor_id_is_valid(motor_id_t motor)
{
    return (motor == MOTOR_LEFT) || (motor == MOTOR_RIGHT);
}

/* 把浮点数限制在指定范围内。 */
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

/* 速度环只关心速度大小，所以把带方向的脉冲数转换为绝对值。 */
static int32_t absolute_pulses(int32_t pulses)
{
    return (pulses < 0) ? -pulses : pulses;
}

/* 根据运动方向设置驱动芯片的一组IN1/IN2引脚。 */
static void set_direction_pins(uint32_t in1_pin,
                               uint32_t in2_pin,
                               motor_direction_t direction)
{
    if (direction == MOTOR_STOP) {
        /* 两个输入同时为高，使用驱动芯片的短刹车模式。 */
        DL_GPIO_setPins(MOTOR_PORT, in1_pin | in2_pin);
    } else if (direction == MOTOR_FORWARD) {
        /* IN1=1、IN2=0。 */
        DL_GPIO_setPins(MOTOR_PORT, in1_pin);
        DL_GPIO_clearPins(MOTOR_PORT, in2_pin);
    } else {
        /* IN1=0、IN2=1。 */
        DL_GPIO_clearPins(MOTOR_PORT, in1_pin);
        DL_GPIO_setPins(MOTOR_PORT, in2_pin);
    }
}

/* 每100ms读取并清空脉冲计数，然后计算左右轮线速度。 */
static void sample_wheel_speeds(void)
{
    motor_id_t motor;

    for (motor = MOTOR_LEFT; motor < MOTOR_COUNT; motor++) {
        /* 先复制再清零，下一周期可以重新累计。 */
        int32_t pulses = s_pending_pulses[motor];

        s_pending_pulses[motor] = 0;
        pulses = absolute_pulses(pulses);
        /* 速度 = 圈数 × 车轮周长 × 每秒采样次数。 */
        s_measured_speed[motor] =
            ((float)pulses / MOTOR_ENCODER_PULSES_PER_REV) *
            MOTOR_PI * MOTOR_WHEEL_DIAMETER_MM * MOTOR_SPEED_SAMPLE_HZ;
    }
}

/* 对指定车轮执行一次带前馈的速度PI计算。 */
static void update_speed_controller(motor_id_t motor)
{
    float target = s_target_speed[motor];
    float measured = s_measured_speed[motor];
    float error;
    float output;

    /* 目标为0时立即清积分和PWM，防止再次启动时积分残留。 */
    if (target <= 0.0f) {
        s_speed_integral[motor] = 0.0f;
        motor_set_pwm(motor, 0U);
        return;
    }

    /* 当前速度误差。 */
    error = target - measured;

    /* 积分用于逐步补偿负载、摩擦和左右电机差异。 */
    s_speed_integral[motor] += error * MOTOR_CONTROL_DT_S;
    s_speed_integral[motor] =
        clamp_float(s_speed_integral[motor],
                    -MOTOR_INTEGRAL_LIMIT,
                    MOTOR_INTEGRAL_LIMIT);

    /* 基础前馈PWM加上PI修正，响应比纯PI更快。 */
    output = MOTOR_PWM_PER_MM_S * target;
    output += MOTOR_KP * error;
    output += MOTOR_KI * s_speed_integral[motor];
    output = clamp_float(output, 0.0f, (float)MOTOR_PWM_MAX);

    motor_set_pwm(motor, (uint16_t)output);
}

void motor_init(void)
{
    motor_id_t motor;

    /* 清空左右轮所有软件状态。 */
    for (motor = MOTOR_LEFT; motor < MOTOR_COUNT; motor++) {
        s_pending_pulses[motor] = 0;
        s_target_speed[motor] = 0.0f;
        s_measured_speed[motor] = 0.0f;
        s_speed_integral[motor] = 0.0f;
        s_pwm[motor] = 0U;
    }

    /* 拉高STBY，使能电机驱动芯片。 */
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_SBYT_PIN);

    /* 初始化时两侧电机保持停止且PWM为0。 */
    motor_set_direction(MOTOR_LEFT, MOTOR_STOP);
    motor_set_direction(MOTOR_RIGHT, MOTOR_STOP);
    motor_set_pwm(MOTOR_LEFT, 0U);
    motor_set_pwm(MOTOR_RIGHT, 0U);

    /* 启动PWM定时器和100ms速度环定时器。 */
    DL_Timer_startCounter(PWM_MOTOR_INST);
    DL_Timer_startCounter(TIMER_0_INST);

    /* 打开左右编码器GPIO中断和速度环定时器中断。 */
    NVIC_ClearPendingIRQ(encoder_INT_IRQN);
    NVIC_EnableIRQ(encoder_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

void motor_set_direction(motor_id_t motor, motor_direction_t direction)
{
    /* 非法编号直接忽略。 */
    if (!motor_id_is_valid(motor)) {
        return;
    }

    /* 左轮直接按照传入方向设置AIN1/AIN2。 */
    if (motor == MOTOR_LEFT) {
        set_direction_pins(MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, direction);
        return;
    }

    /* 右电机接线极性与左电机相反，所以软件中交换前进和后退。 */
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
    /* 防止PWM值超过定时器周期。 */
    if (pwm > MOTOR_PWM_MAX) {
        pwm = MOTOR_PWM_MAX;
    }

    /* 左右轮分别使用PWM定时器的通道0和通道1。 */
    channel = (motor == MOTOR_LEFT) ? GPIO_PWM_MOTOR_C0_IDX
                                    : GPIO_PWM_MOTOR_C1_IDX;
    s_pwm[motor] = pwm;
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, pwm, channel);
}

void motor_set_speed_target(motor_id_t motor, float speed_mm_s)
{
    if (!motor_id_is_valid(motor)) {
        return;
    }
    /* 负速度不在本接口中表示方向，统一按0处理；方向由方向接口设置。 */
    s_target_speed[motor] = (speed_mm_s > 0.0f) ? speed_mm_s : 0.0f;
}

void motor_drive_forward(float left_speed_mm_s, float right_speed_mm_s)
{
    /* 同时设置两侧为前进，然后分别写入左右目标速度。 */
    motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
    motor_set_speed_target(MOTOR_LEFT, left_speed_mm_s);
    motor_set_speed_target(MOTOR_RIGHT, right_speed_mm_s);
}

void motor_stop_all(void)
{
    motor_id_t motor;

    /* 修改多项共享状态时暂时关闭速度环中断，避免中断读到半更新数据。 */
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    for (motor = MOTOR_LEFT; motor < MOTOR_COUNT; motor++) {
        s_pending_pulses[motor] = 0;
        s_target_speed[motor] = 0.0f;
        s_measured_speed[motor] = 0.0f;
        s_speed_integral[motor] = 0.0f;
        motor_set_pwm(motor, 0U);
        motor_set_direction(motor, MOTOR_STOP);
    }
    /* 状态全部清除后重新打开速度环中断。 */
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

/* 以下getter只返回内部保存值，不会改变电机状态。 */
float motor_get_left_speed(void)
{
    return s_measured_speed[MOTOR_LEFT];
}

float motor_get_right_speed(void)
{
    return s_measured_speed[MOTOR_RIGHT];
}

float motor_get_left_target_speed(void)
{
    return s_target_speed[MOTOR_LEFT];
}

float motor_get_right_target_speed(void)
{
    return s_target_speed[MOTOR_RIGHT];
}

uint16_t motor_get_left_pwm(void)
{
    return s_pwm[MOTOR_LEFT];
}

uint16_t motor_get_right_pwm(void)
{
    return s_pwm[MOTOR_RIGHT];
}

void GROUP1_IRQHandler(void)
{
    /* 判断本次GPIO中断来自左编码器A相还是右编码器A相。 */
    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
    case encoder_AA_IIDX:
        /* 左轮A相上升沿到来时读取B相，判断旋转方向并累计脉冲。 */
        s_pending_pulses[MOTOR_LEFT] +=
            DL_GPIO_readPins(GPIOA, encoder_AB_PIN) ? 1 : -1;
        break;

    case encoder_BA_IIDX:
        /* 右轮A相上升沿到来时读取B相，判断旋转方向并累计脉冲。 */
        s_pending_pulses[MOTOR_RIGHT] +=
            DL_GPIO_readPins(GPIOA, encoder_BB_PIN) ? 1 : -1;
        break;

    default:
        break;
    }
}

void TIMA0_IRQHandler(void)
{
    /* 只处理定时器周期到达产生的LOAD中断。 */
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) != DL_TIMER_IIDX_LOAD) {
        return;
    }

    /* 每100ms先测速，再分别更新左右轮速度PI和PWM。 */
    sample_wheel_speeds();
    update_speed_controller(MOTOR_LEFT);
    update_speed_controller(MOTOR_RIGHT);
}
