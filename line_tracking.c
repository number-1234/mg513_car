#include "line_tracking.h"

#include "app_config.h"
#include "line_follow.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

/*
 * 本文件负责“读取灰度传感器 -> 调用原循迹算法 -> 设置左右轮速度”。
 * 灰度采集驱动No_Mcu_Ganv_Grayscale_Sensor.c和循迹算法line_follow.c保持独立。
 */

/* 灰度驱动必须使用的对象，只保存在本文件内部，主函数不需要操作。 */
static No_MCU_Sensor s_line_sensor;

/* 8路灰度传感器放在白色表面时的校准值。 */
static unsigned short s_white_calibration[8] = {
    2521, 3311, 3343, 3153, 2342, 1785, 2634, 2096
};

/* 8路灰度传感器放在黑线表面时的校准值。 */
static unsigned short s_black_calibration[8] = {
    500, 500, 500, 500, 500, 500, 500, 500
};

/* 最近一次检测结果和循迹输出，供简单getter函数读取。 */
static bool s_has_line;                    /* true表示当前检测到黑线 */
static uint8_t s_sensor_bits = 0xFFU;      /* 8路传感器数字量 */
static uint8_t s_left_percent;             /* 左轮循迹速度百分比 */
static uint8_t s_right_percent;            /* 右轮循迹速度百分比 */
static uint16_t s_lost_count;              /* 从有线变为丢线的次数 */

/*
 * 第三方灰度驱动输出的位顺序与LineFollow_Run()需要的左右顺序相反，
 * 因此把bit0~bit7完全翻转后再交给循迹算法。
 */
static uint8_t reverse_sensor_bits(uint8_t value)
{
    uint8_t reversed = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        /* 原来的bit位置有数据，就把它放到对称位置。 */
        if ((value & (uint8_t)(1U << bit)) != 0U) {
            reversed |= (uint8_t)(1U << (7U - bit));
        }
    }
    return reversed;
}

void line_tracking_init(void)
{
    /* 把黑白校准值交给灰度驱动。 */
    No_MCU_Ganv_Sensor_Init(&s_line_sensor,
                            s_white_calibration,
                            s_black_calibration);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN); /* 允许ADC采样中断 */
    LineFollow_Init();                     /* 清除循迹PID历史状态 */

    /* 清除本模块保存的上一次运行结果。 */
    s_has_line = false;
    s_sensor_bits = 0xFFU;
    s_left_percent = 0U;
    s_right_percent = 0U;
    s_lost_count = 0U;
}

bool line_tracking_run(void)
{
    /* 保存进入本函数前的状态，用来判断是否刚刚发生丢线。 */
    bool had_line = s_has_line;

    /* 完成一次8路灰度传感器采样和数字量转换。 */
    No_Mcu_Ganv_Sensor_Task_Without_tick(&s_line_sensor);
    s_sensor_bits =
        reverse_sensor_bits(Get_Digtal_For_User(&s_line_sensor));

    /* 原循迹算法根据黑线位置计算左右轮速度百分比。 */
    LineFollow_Run(s_sensor_bits, &s_left_percent, &s_right_percent);

    /* 当前循迹算法使用左右双零表示“完全没有检测到线”。 */
    if ((s_left_percent == 0U) && (s_right_percent == 0U)) {
        s_has_line = false;

        /* 只在有线->丢线的跳变瞬间计数一次。 */
        if (had_line) {
            s_lost_count++;
        }
        return false; /* 不设置电机，交给straight_drive处理 */
    }

    /* 找到线：把百分比换算成mm/s并立即交给电机速度环。 */
    s_has_line = true;
    motor_drive_forward(
        (float)s_left_percent * APP_LINE_SPEED_FULL_SCALE_MM_S / 100.0f,
        (float)s_right_percent * APP_LINE_SPEED_FULL_SCALE_MM_S / 100.0f);
    return true; /* 已经完成本周期的循迹控制 */
}

/* 以下函数只是返回最近一次数据，不会重新读取传感器。 */
bool line_tracking_has_line(void)
{
    return s_has_line;
}

uint8_t line_tracking_get_sensor_bits(void)
{
    return s_sensor_bits;
}

uint8_t line_tracking_get_left_percent(void)
{
    return s_left_percent;
}

uint8_t line_tracking_get_right_percent(void)
{
    return s_right_percent;
}

uint16_t line_tracking_get_lost_count(void)
{
    return s_lost_count;
}
