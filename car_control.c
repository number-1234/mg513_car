#include "car_control.h"

#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "delay.h"
#include "line_follow.h"
#include "ti_msp_dl_config.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

#if !APP_MOTOR_TEST_MODE
/* 灰度传感器的白色和黑色校准值。 */
static unsigned short s_white_calibration[8] = {
   2521,3311,3343,3153,2342,1785,2634,2096
};
static unsigned short s_black_calibration[8] = {
  500,500,500,500,500,500,500,500
};
static No_MCU_Sensor s_line_sensor;
static uint8_t s_sensor_bits = 0xFFU;
static uint8_t s_left_command_percent;
static uint8_t s_right_command_percent;

/* 将第三方驱动的位顺序转换成循迹算法使用的左到右顺序。 */
static uint8_t reverse_sensor_bits(uint8_t value)
{
    uint8_t reversed = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & (uint8_t)(1U << bit)) != 0U) {
            reversed |= (uint8_t)(1U << (7U - bit));
        }
    }
    return reversed;
}

/* 完成一次传感器读取，并把循迹输出换算成左右轮目标速度。 */
static void update_line_following(void)
{
    uint8_t sensor_bits;
    uint8_t left_percent;
    uint8_t right_percent;

    No_Mcu_Ganv_Sensor_Task_Without_tick(&s_line_sensor);
    sensor_bits = reverse_sensor_bits(Get_Digtal_For_User(&s_line_sensor));
    LineFollow_Run(sensor_bits, &left_percent, &right_percent);

    s_sensor_bits = sensor_bits;
    s_left_command_percent = left_percent;
    s_right_command_percent = right_percent;

    /* 循迹模块返回双零时立即停车。 */
    if ((left_percent == 0U) && (right_percent == 0U)) {
        motor_stop_all();
        return;
    }

    motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
    motor_set_speed_target(
        MOTOR_LEFT,
        (float)left_percent * APP_LINE_SPEED_FULL_SCALE_MM_S / 100.0f);
    motor_set_speed_target(
        MOTOR_RIGHT,
        (float)right_percent * APP_LINE_SPEED_FULL_SCALE_MM_S / 100.0f);
}
#endif

void car_control_init(void)
{
    motor_init();

#if APP_MOTOR_TEST_MODE
    /* 电机测试模式：两侧使用相同目标速度，循迹传感器不参与。 */
    motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
    motor_set_pwm(MOTOR_LEFT, APP_MOTOR_TEST_INITIAL_PWM);
    motor_set_pwm(MOTOR_RIGHT, APP_MOTOR_TEST_INITIAL_PWM);
    motor_set_speed_target(MOTOR_LEFT, APP_MOTOR_TEST_TARGET_MM_S);
    motor_set_speed_target(MOTOR_RIGHT, APP_MOTOR_TEST_TARGET_MM_S);
#else
    /* 循迹模式：初始化灰度传感器和循迹控制器。 */
    No_MCU_Ganv_Sensor_Init(&s_line_sensor,
                            s_white_calibration,
                            s_black_calibration);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    LineFollow_Init();
#endif

    delay_ms(APP_STARTUP_DELAY_MS);
}

void car_control_update(void)
{
#if !APP_MOTOR_TEST_MODE
    /* 电机测试模式下无需执行周期任务，速度环由定时器中断运行。 */
    update_line_following();
#endif
}

void car_control_get_telemetry(car_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    motor_get_telemetry(MOTOR_LEFT, &telemetry->left_motor);
    motor_get_telemetry(MOTOR_RIGHT, &telemetry->right_motor);
#if APP_MOTOR_TEST_MODE
    telemetry->sensor_bits = 0xFFU;
    telemetry->left_command_percent = 0U;
    telemetry->right_command_percent = 0U;
#else
    telemetry->sensor_bits = s_sensor_bits;
    telemetry->left_command_percent = s_left_command_percent;
    telemetry->right_command_percent = s_right_command_percent;
#endif
}
