/**
 * @file    screen.c
 * @brief   三易串口屏数据推送
 *
 * 协议：ASCII 指令 wset <控件> <值>\r\n
 * 复用 UART0（与 printf 同一串口），每 100ms 调用一次 Screen_Update
 *
 * 屏上控件名需要与代码中的名称一致，在屏幕工程里建好同名控件。
 */
#include "screen.h"
#include "ti_msp_dl_config.h"

/* ── 屏幕控件名（按你的屏幕工程修改） ── */
#define SCR_FLAG     "flag"
#define SCR_SENSOR   "sensor"
#define SCR_YAW      "yaw"
#define SCR_L_VEL    "l_vel"
#define SCR_R_VEL    "r_vel"
#define SCR_L_PWM    "l_pwm"
#define SCR_R_PWM    "r_pwm"

/* 发送一个字节到串口（复用 printf 的 fputc 逻辑） */
static void scr_putc(char c)
{
    while (DL_UART_isBusy(UART_0_INST)) {}
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)c);
}

static void scr_send(const char *str)
{
    while (*str) scr_putc(*str++);
}

void Screen_Update(int flag, uint8_t sensor, float yaw,
                   float l_vel, float r_vel,
                   int l_pwm, int r_pwm)
{
    char buf[32];

    /* — 每条 wset 指令以 \r\n 结尾 — */

    // flag
    snprintf(buf, sizeof(buf), "wset " SCR_FLAG ".val %d\r\n", flag);
    scr_send(buf);

    // sensor (16进制)
    snprintf(buf, sizeof(buf), "wset " SCR_SENSOR ".txt %02X\r\n", sensor);
    scr_send(buf);

    // yaw
    snprintf(buf, sizeof(buf), "wset " SCR_YAW ".val %.1f\r\n", (double)yaw);
    scr_send(buf);

    // left velocity
    snprintf(buf, sizeof(buf), "wset " SCR_L_VEL ".val %.1f\r\n", (double)l_vel);
    scr_send(buf);

    // right velocity
    snprintf(buf, sizeof(buf), "wset " SCR_R_VEL ".val %.1f\r\n", (double)r_vel);
    scr_send(buf);

    // left pwm
    snprintf(buf, sizeof(buf), "wset " SCR_L_PWM ".val %d\r\n", l_pwm);
    scr_send(buf);

    // right pwm
    snprintf(buf, sizeof(buf), "wset " SCR_R_PWM ".val %d\r\n", r_pwm);
    scr_send(buf);
}
