 #include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "Control/control.h"
#include "Encoder/Encoder.h"
#include "GYRO/GYRO_JY901.h"
#include "App/app_mode.h"
#include "Key/key.h"
#include "Sensor/Sensor.h"
#include "sys/sys.h"
#include "LCD/lcd_init.h"
#include "LCD/lcd.h"

volatile int flag = 2;  /* 初始停车 */

/* 实测：串口累计 50 mm 时小车实际约行驶 200 mm。 */
#define DISTANCE_CALIBRATION_SCALE  0.93f

/* SysConfig 的 TIMA0 控制定时器为 10ms；编码器仍按 100ms 汇总速度。 */
#define ENCODER_SAMPLE_TICKS        10U

static uint32_t s_last_print_ms;
static uint32_t s_last_dist_ms;
static float s_dist_mm = 0;   /* 累计距离 mm */
static uint32_t s_motor_start_ms;
static uint32_t s_motor_run_ms;
static bool s_motor_running;
static uint8_t s_encoder_sample_tick;
static uint8_t s_lcd_page;

static void LCD_Show_Mode_Menu(void)
{
    if (s_lcd_page == 1U) {
        return;
    }

    s_lcd_page = 1U;
    LCD_Fill(0, 0, 240, 280, BLACK);
    LCD_ShowString(42, 22, "SELECT MODE", YELLOW, BLACK, 16, 0);
    LCD_ShowString(20, 75, "PB15: FOLLOW", GREEN, BLACK, 16, 0);
    LCD_ShowString(20, 115, "PB16: 2M", CYAN, BLACK, 16, 0);
    LCD_ShowString(20, 155, "PB2: SLOW", CYAN, BLACK, 16, 0);
    LCD_ShowString(20, 220, "PRESS KEY", WHITE, BLACK, 16, 0);
}

static void Update_Motor_Run_Time(void)
{
    if ((Motor_Left != 0) || (Motor_Right != 0)) {
        if (!s_motor_running) {
            s_motor_running = true;
            s_motor_start_ms = system_millis();
            s_motor_run_ms = 0U;
        } else {
            s_motor_run_ms = (uint32_t)(system_millis() - s_motor_start_ms);
        }
    } else {
        /* 电机停止时保留最终时间；下一次启动才重新从 0 开始计时。 */
        s_motor_running = false;
    }
}

static void Update_Distance(uint32_t now_ms)
{
    float dt_s = (float)(uint32_t)(now_ms - s_last_dist_ms) / 1000.0f;
    float average_speed_mm_s = (EncoderA_VEL + EncoderB_VEL) * 0.5f;

    s_dist_mm += average_speed_mm_s * dt_s * DISTANCE_CALIBRATION_SCALE;
    s_last_dist_ms = now_ms;
}

static void Print_Running_Data(void)
{
    printf("Flag:%d Sen:%02X ADC8:%s Yaw:%.1f "
           "L:%.1f(%ld) R:%.1f(%ld) PWM:%d,%d Dist:%.0fmm\r\n",
           flag,
           (unsigned int)Sensor_Bits,
           Sensor_Is_Online() ? "OK" : "ERR",
           (double)Yaw,
           (double)EncoderA_VEL, (long)EncoderA_Last_Raw,
           (double)EncoderB_VEL, (long)EncoderB_Last_Raw,
           Motor_Left,
           Motor_Right,
           (double)s_dist_mm);
}

static void LCD_Show_Data(void)
{
#if 0
    char buf[16];
    static int last_flag=-1, last_pwm_l=-1, last_pwm_r=-1;
    if (s_lcd_page != 2U) {
        s_lcd_page = 2U;
        LCD_Fill(0, 0, 240, 280, BLACK);
        LCD_ShowString(48, 45, "TIME", YELLOW, BLACK, 32, 0);
        LCD_ShowString(170, 145, "s", YELLOW, BLACK, 32, 0);
    }

    static bool first = true;

    /* 首次画静态文字 */
    if (first) {
        first = false;
        LCD_Fill(0,0,240,280,BLACK);
        LCD_ShowString(15,2,"MG513 CAR",YELLOW,BLACK,16,0);
        LCD_ShowString(15,25,"Flag:",WHITE,BLACK,16,0);
        LCD_ShowString(15,50,"Sen:",WHITE,BLACK,16,0);
        LCD_ShowString(15,75,"Yaw:",WHITE,BLACK,16,0);
        LCD_ShowString(15,105,"L:",WHITE,BLACK,16,0);
        LCD_ShowString(120,105,"mm/s",BLUE,BLACK,12,0);
        LCD_ShowString(15,130,"R:",WHITE,BLACK,16,0);
        LCD_ShowString(120,130,"mm/s",BLUE,BLACK,12,0);
        LCD_ShowString(15,160,"L_PWM:",WHITE,BLACK,16,0);
        LCD_ShowString(15,205,"R_PWM:",WHITE,BLACK,16,0);
        LCD_ShowString(15,250,"Run:",WHITE,BLACK,16,0);
    }

    /* 动态数据：背景色覆盖旧值再写新值（无闪烁） */
    if (flag != last_flag) {
        snprintf(buf,8,"%d",flag);
        LCD_ShowString(70,25,(u8*)buf,GREEN,BLACK,16,0);
        last_flag = flag;
    }

    snprintf(buf,8,"%02X",(unsigned int)Sensor_Bits);
    LCD_ShowString(70,50,(u8*)buf,CYAN,BLACK,16,0);

    LCD_ShowFloatNum1(70,75,Yaw,5,RED,BLACK,16);
    LCD_ShowFloatNum1(50,105,EncoderA_VEL,4,WHITE,BLACK,16);
    LCD_ShowFloatNum1(50,130,EncoderB_VEL,4,WHITE,BLACK,16);

    if (Motor_Left != last_pwm_l) {
        LCD_ShowIntNum(90,160,Motor_Left,4,WHITE,BLACK,16);
        LCD_Fill(15,181,244,194,BLACK);
        LCD_DrawRectangle(15,180,245,195,GRAY);
        if(Motor_Left>0) LCD_Fill(15,180,5+Motor_Left/4,195,GREEN);
        last_pwm_l = Motor_Left;
    }

    if (Motor_Right != last_pwm_r) {
        LCD_ShowIntNum(90,205,Motor_Right,4,WHITE,BLACK,16);
        LCD_Fill(15,226,244,239,BLACK);
        LCD_DrawRectangle(15,225,245,240,GRAY);
        if(Motor_Right>0) LCD_Fill(15,225,5+Motor_Right/4,240,GREEN);
        last_pwm_r = Motor_Right;
    }

    LCD_ShowFloatNum1(70,250,(float)s_motor_run_ms / 1000.0f,
                      5, YELLOW, BLACK, 16);
    LCD_ShowString(125,250,"s",YELLOW,BLACK,16,0);
#endif

    if (s_lcd_page != 2U) {
        s_lcd_page = 2U;
        LCD_Fill(0, 0, 240, 280, BLACK);
        LCD_ShowString(48, 45, "TIME", YELLOW, BLACK, 32, 0);
        LCD_ShowString(170, 145, "s", YELLOW, BLACK, 32, 0);
    }

    static bool first = true;

    /* LCD 只显示运行时间；停车后保留最终读数。 */
    if (first) {
        first = false;
        LCD_Fill(0, 0, 240, 280, BLACK);
        LCD_ShowString(48, 45, "TIME", YELLOW, BLACK, 32, 0);
        LCD_ShowString(170, 145, "s", YELLOW, BLACK, 32, 0);
    }

    LCD_ShowFloatNum1(30, 125,
                      (float)AppMode_GetDisplayTime(s_motor_run_ms) / 1000.0f,
                      5, CYAN, BLACK, 32);
}

static void LCD_Show_Distance(float distance_mm)
{
    if (s_lcd_page != 3U) {
        s_lcd_page = 3U;
        LCD_Fill(0, 0, 240, 280, BLACK);
        LCD_ShowString(30, 45, "DISTANCE", YELLOW, BLACK, 24, 0);
        LCD_ShowString(165, 145, "mm", YELLOW, BLACK, 24, 0);
    }

    if (distance_mm > 9999.0f) {
        distance_mm = 9999.0f;
    }
    LCD_ShowIntNum(20, 125, (uint16_t)(distance_mm + 0.5f),
                   4, CYAN, BLACK, 32);
}

int main(void)
{

    SYSCFG_DL_init();
    delay_ms(100U);

    LCD_Init();

    Sensor_Init();
    /* 上电即启动一次 ADC 灰度采样；之后主循环始终持续采样，与按键无关。 */
    Sensor_Read();
    Control_Init();
    Key_Init();
    AppMode_Init();

    /* 暂不使用 WIT 陀螺仪，避免它访问 I2C 总线。 */
#if 0
    uint8_t gyro_status = GYRO_Init();
#endif
    Encoder_Init();

#if 0
    if (gyro_status != 0U) {
        LCD_ShowString(40,180,"GYRO FAIL",RED,BLACK,16,0);
    }
#endif

    delay_ms(500);
    flag = 2;
    Sensor_Enable_Stop_Line_After(3000U); /* 起跑后的前 3 秒忽略停车线。 */
    s_motor_running = false;
    s_motor_run_ms = 0U;
    s_encoder_sample_tick = 0U;
    s_last_print_ms = system_millis();
    s_last_dist_ms = s_last_print_ms;
    Sensor_Disable_Stop_Line();
    LCD_Show_Mode_Menu();

    while (1) {
        uint32_t now_ms = system_millis();

        Update_Distance(now_ms);
        /* 先刷新 ADC 循迹状态；按键启动时可立即用当前线位初始化控制器。 */
        Sensor_Read();  /* ADC 八路灰度：0=黑线，1=白底。 */
        AppMode_HandleKey(Key_GetPressed(), s_dist_mm);
        Update_Motor_Run_Time();
        /* PB15 的停车线会停车；PB2 的同一条线只记录时间，车辆不停。 */
        AppMode_Update(s_dist_mm, s_motor_run_ms);
        /* 只做八路循迹：不运行丢线转向状态机，不依赖陀螺仪。 */

        if ((uint32_t)(system_millis() - s_last_print_ms) >= 100U) {
            s_last_print_ms = system_millis();
            Print_Running_Data();
            if (AppMode_UsesDistanceScreen()) {
                LCD_Show_Distance(AppMode_GetDistanceProgress(s_dist_mm));
            } else if (AppMode_UsesTimeScreen()) {
                LCD_Show_Data();
            } else {
                LCD_Show_Mode_Menu();
            }
        }
    }
}

void TIMA0_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_LOAD) {
        /* 10ms 控制；每 10 个控制周期（100ms）更新一次编码器速度。 */
        s_encoder_sample_tick++;
        if (s_encoder_sample_tick >= ENCODER_SAMPLE_TICKS) {
            s_encoder_sample_tick = 0U;
            Read_Encoder();
        }
        Control();
    }
}

void GROUP1_IRQHandler(void)
{
    Encodering();
}
