/**
 * @file    control.c
 * @brief   小车运动控制 — flag=1循迹 / 2停车 / 3转向 / 4直行
 *
 * flag=3: Turn_Run(要转的角度) — 每次传相同目标，到位返回true
 * flag=4: Straight_Run(目标角度, 前进速度) — 直接输出左右轮目标
 */
#include "control.h"
#include "angle_hold.h"
#include "Encoder/Encoder.h"
#include "GYRO/GYRO_JY901.h"
#include "Sensor/Sensor.h"
#include "sys/sys.h"

/* ── 参数 ── */
#define PWM_MAX              1000.0f
#define LINE_CENTER_DEADBAND     3.0f   /* X4/X5 的单探头切换均视为居中 */
#define LINE_FILTER_ALPHA        0.28f  /* 小偏差平滑，抑制直线传感器跳变 */
#define LINE_OUTER_FILTER_ALPHA  0.75f  /* 外侧压线时提高响应速度，但仍保留平滑 */
/* Sensor_Bits: bit7..bit0 = X1..X8，0 表示压到黑线。 */
#define LINE_OUTER_SENSOR_MASK   0xC3U /* X1、X2、X7、X8 */
/* PB2 环形赛道：直线 1.5m，两个 r=0.5m 的半圆。 */
#define PB2_TRACK_LAP_MM            6141.6f
#define PB2_FIRST_CURVE_START_MM    1500.0f
#define PB2_SECOND_CURVE_START_MM   4570.8f
#define PB2_CURVE_PREPARE_MM         220.0f
#define PB2_CURVE_HANDOFF_MM         180.0f
#define START_SWAY_HALF_PERIOD_MS     100U
#define STRAIGHT_SPEED_MM_S  150.0f   /* 直行速度 */
#define STRAIGHT_P            10.0f   /* 直行角度 P */
#define STRAIGHT_D            10.0f   /* 直行角度 D */
#define SPEED_A_KP             0.25f
#define SPEED_A_KI             0.50f
#define SPEED_A_FEEDFORWARD    0.58f
#define SPEED_B_KP             0.38f
#define SPEED_B_KI             0.22f
#define SPEED_B_FEEDFORWARD    0.72f
#define SPEED_DT               0.01f   /* 控制定时器：10ms */
#define SPEED_A_INTEGRAL_MAX    400.0f
#define SPEED_B_INTEGRAL_MAX    220.0f

/* ── 全局 ── */
extern volatile int flag;
volatile int   Motor_Left, Motor_Right;
volatile float Turn_Zero_Yaw, Straight_Zero_Yaw;

/* ── 静态 ── */
static float s_i_l, s_i_r;
static float s_last_dev;
static float s_filtered_dev;
static float s_outer_blend;
static float s_limited_base_speed;
static uint32_t s_line_start_ms;
static volatile float s_track_distance_mm;

typedef struct {
    float line_speed;
    float curve_speed;
    float max_speed;
    float line_p;
    float curve_p;
    float line_d;
    float max_bias;
    float outer_bias_gain;
    float outer_transition_step;
    float outer_release_step;
    float soft_start_speed;
    uint32_t soft_start_ms;
    float speed_accel_step;
    float speed_decel_step;
    float linear_accel_mm_s2;
    float launch_speed;
    uint32_t launch_ms;
    uint32_t start_sway_ms;
    float start_sway_bias;
    bool average_line_position;
} LineProfile;

/* PB15: 快速循迹参数。 */
static const LineProfile s_fast_line_profile = {
    450.0f, 300.0f, 500.0f, 3.6f, 7.5f, 0.3f, 70.0f, 1.7f, 1.0f, 1.0f,
    80.0f, 1600U, 4.0f, 4.0f, 0.0f, 0.0f, 0U, 0U, 0.0f, false
};

/* PB2: 持续循迹模式；停车线仅由 App 用于记录时间。 */
static const LineProfile s_slow_follow_line_profile = {
    340.0f, 210.0f, 510.0f, 5.0f, 6.0f, 0.18f, 55.0f, 1.45f, 0.12f, 0.05f,
    /* 前 0.9 秒先平滑到 60mm/s，随后以 26mm/s² 匀加速。 */
    0.0f, 0U, 0.28f, 3.0f, 26.0f, 60.0f, 900U, 400U, 28.0f, true
};

/* PB16: 约 2m 模式，低速稳定循迹，参数与 PB15 独立。 */
static const LineProfile s_distance_line_profile = {
    260.0f, 210.0f, 350.0f, 3.0f, 5.5f, 0.15f, 45.0f, 1.2f, 0.25f, 0.12f,
    /* PB16 携球低速模式：从 0 起步，3 秒缓慢爬升。 */
    0.0f, 3000U, 0.8f, 0.8f, 0.0f, 0.0f, 0U, 400U, 28.0f, true
};

static const LineProfile *s_line_profile = &s_fast_line_profile;

/* 将当前 8 路状态换算为循迹偏差；中心两探头的来回跳变视为居中。 */
static float Control_ReadLineDeviation(bool average_position)
{
    float dev;

    if (average_position) {
        uint8_t black_count = 0U;

        dev = 0.0f;
        if ((Sensor_Bits & 0x80U) == 0U) { dev -= 12.0f; black_count++; }
        if ((Sensor_Bits & 0x40U) == 0U) { dev -=  9.0f; black_count++; }
        if ((Sensor_Bits & 0x20U) == 0U) { dev -=  7.0f; black_count++; }
        if ((Sensor_Bits & 0x10U) == 0U) { dev -=  3.0f; black_count++; }
        if ((Sensor_Bits & 0x08U) == 0U) { dev +=  3.0f; black_count++; }
        if ((Sensor_Bits & 0x04U) == 0U) { dev +=  7.0f; black_count++; }
        if ((Sensor_Bits & 0x02U) == 0U) { dev +=  9.0f; black_count++; }
        if ((Sensor_Bits & 0x01U) == 0U) { dev += 12.0f; black_count++; }

        if (black_count > 0U) {
            dev /= (float)black_count;
        }
    } else {
        dev = (float)Incremental_Quantity();
    }

    if ((dev >= -LINE_CENTER_DEADBAND) &&
        (dev <= LINE_CENTER_DEADBAND)) {
        dev = 0.0f;
    }

    return dev;
}

/* 外侧四个探头任一压线时，说明偏离较大，需要快速修正。 */
static bool Control_HasOuterLine(void)
{
    return (Sensor_Bits & LINE_OUTER_SENSOR_MASK) != LINE_OUTER_SENSOR_MASK;
}

/* 在半圆入口前预降速；不按里程直接转向，转向始终由灰度传感器决定。 */
static float Control_GetTrackCurveBlend(const LineProfile *profile)
{
    float phase;
    float curve_start[2] = {
        PB2_FIRST_CURVE_START_MM,
        PB2_SECOND_CURVE_START_MM
    };

    if (profile != &s_slow_follow_line_profile) {
        return 0.0f;
    }

    phase = s_track_distance_mm;
    while (phase >= PB2_TRACK_LAP_MM) {
        phase -= PB2_TRACK_LAP_MM;
    }

    for (uint8_t i = 0U; i < 2U; i++) {
        if ((phase >= (curve_start[i] - PB2_CURVE_PREPARE_MM)) &&
            (phase < curve_start[i])) {
            return (phase - (curve_start[i] - PB2_CURVE_PREPARE_MM)) /
                   PB2_CURVE_PREPARE_MM;
        }

        /* 进弯后保留一小段预判，直到外侧灰度探头接管。 */
        if ((phase >= curve_start[i]) &&
            (phase < (curve_start[i] + PB2_CURVE_HANDOFF_MM))) {
            return 1.0f - (phase - curve_start[i]) /
                   PB2_CURVE_HANDOFF_MM;
        }
    }

    return 0.0f;
}

void Control_SetLineProfile(ControlLineProfile profile)
{
    if (profile == CONTROL_LINE_PROFILE_DISTANCE) {
        s_line_profile = &s_distance_line_profile;
    } else if (profile == CONTROL_LINE_PROFILE_SLOW_FOLLOW) {
        s_line_profile = &s_slow_follow_line_profile;
    } else {
        s_line_profile = &s_fast_line_profile;
    }

    /* 每次按键启动模式时，从低速平滑爬升。 */
    s_line_start_ms = system_millis();
    s_limited_base_speed = 0.0f;
}

void Control_SetTrackDistance(float distance_mm)
{
    s_track_distance_mm = (distance_mm > 0.0f) ? distance_mm : 0.0f;
}

/* ================================================================
 *  Turn_Run(target_deg)
 *    传入目标角度（°），重复调用。到位返回 true。
 *    例: Turn_Run(Yaw + 90)  — 原地右转90°
 * ================================================================ */
static bool Turn_Run(float target_deg)
{
    static bool inited;
    if (!inited) { AngleHold_SetTarget(target_deg); inited = true; }

    AngleHold_Control();
    if (AngleHold_IsArrived(2.0f)) { inited = false; return true; }
    return false;
}

/* ================================================================
 *  Straight_Run(target_deg, speed, *ta, *tb)
 *    传入目标角度和前进速度，输出左右轮目标。
 *    例: Straight_Run(90, 100, &ta, &tb) — 保持90°方向、100mm/s直行
 * ================================================================ */
static void Straight_Run(float target_deg, float base_speed,
                         float *ta, float *tb)
{
    static float prev_yaw;
    float err   = normalize_angle(target_deg - Yaw);
    float delta = Yaw - prev_yaw;
    prev_yaw = Yaw;

    float steer = err * STRAIGHT_P - delta * STRAIGHT_D;
    steer = PWM_Limit(steer, base_speed, -base_speed);

    *ta = base_speed - steer;
    *tb = base_speed + steer;
}

/* ================================================================
 *  SpeedPI — 速度闭环
 * ================================================================ */
static float SpeedPI(float *integral, float target, float actual,
                     float kp, float ki, float feedforward,
                     float integral_max)
{
    if (target <= 0.0f) { *integral = 0.0f; return 0.0f; }
    float err = target - actual;
    *integral += err * SPEED_DT;
    *integral  = PWM_Limit(*integral, integral_max, -integral_max);
    return feedforward * target + kp * err + ki * (*integral);
}

/* ================================================================
 *  初始化
 * ================================================================ */
void Control_Init(void)
{
    Motor_Left = Motor_Right = 0;
    Turn_Zero_Yaw = Straight_Zero_Yaw = 0.0f;
    s_i_l = s_i_r = 0.0f;
    s_last_dev = 0.0f;
    s_filtered_dev = 0.0f;
    s_outer_blend = 0.0f;
    s_limited_base_speed = 0.0f;
    s_track_distance_mm = 0.0f;

    /* 按键启动前已经完成一次采样时，直接以当前线位作为滤波初值。
     * 避免由 0 缓慢滤到实际偏差，启动瞬间先直走后才修正。 */
    if (Sensor_Has_Line()) {
        s_filtered_dev =
            Control_ReadLineDeviation(s_line_profile->average_line_position);
        s_last_dev = s_filtered_dev;
    }
    Set_Pwm(0, 0);
}

/* ================================================================
 *  主控制 (每10ms)
 * ================================================================ */
void Control(void)
{
    float ta = 0, tb = 0;

    if (flag == 1) {
        float dev;
        float delta_dev;
        bool outer_line_active = false;
        bool line_valid = false;
        const LineProfile *profile = s_line_profile;
        float track_curve_blend = Control_GetTrackCurveBlend(profile);
        float curve_blend;

        if (Sensor_Has_Line()) {
            dev = Control_ReadLineDeviation(profile->average_line_position);
            outer_line_active = Control_HasOuterLine();
            line_valid = true;

            /* 入弯与出弯分开设置过渡速度，避免出弯接直线时突变。 */
            if (outer_line_active) {
                s_outer_blend += profile->outer_transition_step;
            } else {
                s_outer_blend -= profile->outer_release_step;
            }
            s_outer_blend = PWM_Limit(s_outer_blend, 1.0f, 0.0f);

            /*
             * 直线时数字探头会在相邻位置之间跳变。小偏差先平滑，
             * 中间四个探头(X3~X6)只做缓慢修正；外侧探头压线时
             * 渐进提高滤波响应，避免切到外侧时偏差突然跳变。
             */
            curve_blend = s_outer_blend;
            if (track_curve_blend > curve_blend) {
                curve_blend = track_curve_blend;
            }
            float filter_alpha = LINE_FILTER_ALPHA + curve_blend *
                (LINE_OUTER_FILTER_ALPHA - LINE_FILTER_ALPHA);
            s_filtered_dev += filter_alpha * (dev - s_filtered_dev);

            dev = s_filtered_dev;
            delta_dev = dev - s_last_dev;
            s_last_dev = dev;

        } else {
            /*
             * 0xFF 表示暂时丢线。保持最后一次偏差继续寻找，
             * 但不计算 D，避免偏差在“最后值 -> 0 -> 新值”之间跳变。
             */
            dev = s_last_dev;
            delta_dev = 0.0f;
        }

        /* X1（左）压线时让右轮更快，车向左回正。 */
        float target_speed = profile->line_speed;
        float base_speed;
        float line_p = profile->line_p;
        float bias;
        float outer_bias_gain = 1.0f;

        curve_blend = s_outer_blend;
        if (track_curve_blend > curve_blend) {
            curve_blend = track_curve_blend;
        }

        if (line_valid) {
            target_speed += curve_blend *
                (profile->curve_speed - profile->line_speed);
            line_p += curve_blend * (profile->curve_p - profile->line_p);
            outer_bias_gain += curve_blend *
                (profile->outer_bias_gain - 1.0f);
        }

        /* PB2 使用全程线性加速；弯道仅作为速度上限，不能突然加速。 */
        uint32_t start_elapsed_ms =
            (uint32_t)(system_millis() - s_line_start_ms);
        if (profile->linear_accel_mm_s2 > 0.0f) {
            float linear_speed;

            if ((profile->launch_ms > 0U) &&
                (profile->launch_speed > 0.0f)) {
                if (start_elapsed_ms < profile->launch_ms) {
                    linear_speed = profile->launch_speed *
                        (float)start_elapsed_ms / (float)profile->launch_ms;
                } else {
                    linear_speed = profile->launch_speed +
                        profile->linear_accel_mm_s2 *
                        ((float)(start_elapsed_ms - profile->launch_ms) *
                         0.001f);
                }
            } else {
                linear_speed = profile->linear_accel_mm_s2 *
                               ((float)start_elapsed_ms * 0.001f);
            }

            if (linear_speed > target_speed) {
                linear_speed = target_speed;
            }
            base_speed = linear_speed;
        } else if (start_elapsed_ms < profile->soft_start_ms) {
            float ratio = (float)start_elapsed_ms /
                          (float)profile->soft_start_ms;
            base_speed = profile->soft_start_speed +
                         (target_speed - profile->soft_start_speed) * ratio;
        } else {
            base_speed = target_speed;
        }

        /* 出弯慢加、入弯快减：兼顾球的稳定和弯道循迹。 */
        float accel_step = profile->speed_accel_step;
        if ((start_elapsed_ms < profile->launch_ms) &&
            (profile->launch_ms > 0U) &&
            (profile->launch_speed > 0.0f)) {
            float launch_step = profile->launch_speed * SPEED_DT * 1000.0f /
                                (float)profile->launch_ms;
            if (launch_step > accel_step) {
                accel_step = launch_step;
            }
        }

        if (base_speed > (s_limited_base_speed + accel_step)) {
            s_limited_base_speed += accel_step;
        } else if (base_speed < (s_limited_base_speed - profile->speed_decel_step)) {
            s_limited_base_speed -= profile->speed_decel_step;
        } else {
            s_limited_base_speed = base_speed;
        }
        base_speed = s_limited_base_speed;

        bias = dev * line_p + delta_dev * profile->line_d;

        /* 起步先小幅交替驱动左右轮，预稳球体；不增加纵向目标速度。 */
        if ((start_elapsed_ms < profile->start_sway_ms) &&
            (profile->start_sway_bias > 0.0f)) {
            if (((start_elapsed_ms / START_SWAY_HALF_PERIOD_MS) & 1U) == 0U) {
                bias -= profile->start_sway_bias;
            } else {
                bias += profile->start_sway_bias;
            }
        }

        bias = PWM_Limit(bias, profile->max_bias, -profile->max_bias);
        if (bias >= 0.0f) {
            /* 向右转向修正：左轮为外侧；仅大偏差时额外加速。 */
            ta = PWM_Limit(base_speed + bias * outer_bias_gain,
                           profile->max_speed, 0);
            tb = PWM_Limit(base_speed - bias, profile->max_speed, 0);
        } else {
            /* 向左转向修正：右轮为外侧；仅大偏差时额外加速。 */
            ta = PWM_Limit(base_speed + bias, profile->max_speed, 0);
            tb = PWM_Limit(base_speed - bias * outer_bias_gain,
                           profile->max_speed, 0);
        }
    }
    else if (flag == 2) {
        /* 停车 */
    }
    else if (flag == 3) {
        if (Turn_Run(Yaw - 90)) flag = 4;   /* 转到位切直行 */
        return;
    }
    else if (flag == 4) {
        Straight_Run(0, STRAIGHT_SPEED_MM_S, &ta, &tb);
    }

    Motor_Left = (int)PWM_Limit(
        SpeedPI(&s_i_l, ta, EncoderA_VEL,
                SPEED_A_KP, SPEED_A_KI, SPEED_A_FEEDFORWARD,
                SPEED_A_INTEGRAL_MAX),
        PWM_MAX, 0);
    Motor_Right = (int)PWM_Limit(
        SpeedPI(&s_i_r, tb, EncoderB_VEL,
                SPEED_B_KP, SPEED_B_KI, SPEED_B_FEEDFORWARD,
                SPEED_B_INTEGRAL_MAX),
        PWM_MAX, 0);
    Set_Pwm(Motor_Left, Motor_Right);
}

/* ================================================================
 *  工具 / 兼容旧接口
 * ================================================================ */
void Control_Stop(void)
{
    s_i_l = s_i_r = 0.0f;
    s_outer_blend = 0.0f;
    s_limited_base_speed = 0.0f;
    Motor_Left = Motor_Right = 0;
    Set_Pwm(0, 0);
}
float PID_A(float e, float t)
{
    return SpeedPI(&s_i_l, t, e,
                   SPEED_A_KP, SPEED_A_KI, SPEED_A_FEEDFORWARD,
                   SPEED_A_INTEGRAL_MAX);
}
float PID_B(float e, float t)
{
    return SpeedPI(&s_i_r, t, e,
                   SPEED_B_KP, SPEED_B_KI, SPEED_B_FEEDFORWARD,
                   SPEED_B_INTEGRAL_MAX);
}
float GYRO_Control(float n, float t) { return 0; }
float PWM_Limit(float v, float max, float min) { return limit_float(v, max, min); }
float Control_Get_Angle(void) { return 0; }
float Control_Get_Bias(void)  { return 0; }

/* ================================================================
 *  4路PWM
 * ================================================================ */
void Set_Pwm(int Left, int Right)
{
    Left  = (int)PWM_Limit(Left,  PWM_MAX, -PWM_MAX);
    Right = (int)PWM_Limit(Right, PWM_MAX, -PWM_MAX);

    if (Left > 0) {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, (uint32_t)Left,  GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,              GPIO_PWM_0_C0_IDX);
    } else if (Left < 0) {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, 0,              GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)(-Left), GPIO_PWM_0_C0_IDX);
    } else {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, (uint32_t)PWM_MAX, GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)PWM_MAX, GPIO_PWM_0_C0_IDX);
    }

    if (Right > 0) {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)Right, GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, 0,               GPIO_PWM_1_C1_IDX);
    } else if (Right < 0) {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,               GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, (uint32_t)(-Right), GPIO_PWM_1_C1_IDX);
    } else {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)PWM_MAX, GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, (uint32_t)PWM_MAX, GPIO_PWM_1_C1_IDX);
    }
}
