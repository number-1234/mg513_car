 #include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "Control/control.h"
#include "Encoder/Encoder.h"
#include "GYRO/GYRO_JY901.h"
#include "Sensor/Sensor.h"
#include "sys/sys.h"
#include "LCD/lcd_init.h"
#include "LCD/lcd.h"

volatile int flag = 2;  /* 初始停车 */

static uint32_t s_last_print_ms;
static float s_dist_mm = 0;   /* 累计距离 mm */

static void Print_Running_Data(void)
{
    printf("Flag:%d Sensor:%02X Yaw:%.1f "
           "L:%.1f R:%.1f PWM:%d,%d Dist:%.0fmm\r\n",
           flag,
           (unsigned int)Sensor_Bits,
           (double)Yaw,
           (double)EncoderA_VEL,
           (double)EncoderB_VEL,
           Motor_Left,
           Motor_Right,
           (double)s_dist_mm);
}

static void LCD_Show_Data(void)
{
    char buf[16];
    static int last_flag=-1, last_pwm_l=-1, last_pwm_r=-1;
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
}
int main(void)
{
    uint8_t gyro_status;

    SYSCFG_DL_init();
    delay_ms(100U);

    LCD_Init();
    LCD_Fill(0,0,240,280,BLACK);
    LCD_ShowString(20,120,"MG513 CAR",YELLOW,BLACK,24,0);
    LCD_ShowString(40,155,"INIT...",WHITE,BLACK,16,0);

    Sensor_Init();
    Control_Init();

    gyro_status = GYRO_Init();
    Encoder_Init();

    if (gyro_status != 0U) {
        LCD_ShowString(40,180,"GYRO FAIL",RED,BLACK,16,0);
    }

    delay_ms(500);
    flag = 1;
    s_last_print_ms = system_millis();

    while (1) {
        Sensor_Read();
        (void)GYRO_Update();
        Follow_Route();

        if ((uint32_t)(system_millis() - s_last_print_ms) >= 100U) {
            s_last_print_ms = system_millis();
            s_dist_mm += (EncoderA_VEL + EncoderB_VEL) * 0.05f;
            Print_Running_Data();
            LCD_Show_Data();
        }
    }
}

void TIMA0_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_LOAD) {
        Read_Encoder();
        Control();
    }
}

void GROUP1_IRQHandler(void)
{
    Encodering();
}
