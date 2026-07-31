#include "app_mode.h"

#include "Control/control.h"
#include "Sensor/Sensor.h"

extern volatile int flag;

static AppMode s_mode;
static float s_distance_start_mm;
static bool s_show_distance;
static bool s_stop_time_recorded;
static uint32_t s_stop_time_ms;

#define FOLLOW_DISTANCE_MM 2000.0f

static void AppMode_StartFollow(void)
{
    /* 每次启动清空速度 PI，防止带入上一次运行的积分。 */
    Control_SetLineProfile(CONTROL_LINE_PROFILE_FAST);
    Control_Init();
    Sensor_Enable_Stop_Line_After(3000U);
    s_stop_time_recorded = false;
    s_stop_time_ms = 0U;
    s_mode = APP_MODE_FOLLOW_STOP_LINE;
    s_show_distance = false;
    flag = 1;
}

static void AppMode_StartSlowFollow(float distance_mm)
{
    Control_SetLineProfile(CONTROL_LINE_PROFILE_SLOW_FOLLOW);
    Control_Init();
    /* PB2 通过停车线只记录时间，车辆保持持续循迹。 */
    Sensor_Enable_Stop_Line_After(3000U);
    s_distance_start_mm = distance_mm;
    Control_SetTrackDistance(0.0f);
    s_stop_time_recorded = false;
    s_stop_time_ms = 0U;
    s_mode = APP_MODE_FOLLOW_CONTINUOUS;
    s_show_distance = false;
    flag = 1;
}

static void AppMode_StartDistanceFollow(float distance_mm)
{
    Control_SetLineProfile(CONTROL_LINE_PROFILE_DISTANCE);
    Control_Init();
    Sensor_Disable_Stop_Line();
    s_stop_time_recorded = false;
    s_stop_time_ms = 0U;
    s_distance_start_mm = distance_mm;
    s_mode = APP_MODE_FOLLOW_DISTANCE;
    s_show_distance = true;
    flag = 1;
}

void AppMode_Init(void)
{
    s_mode = APP_MODE_MENU;
    s_show_distance = false;
    s_stop_time_recorded = false;
    s_stop_time_ms = 0U;
    Sensor_Disable_Stop_Line();
    flag = 2;
    Control_Stop();
}

void AppMode_HandleKey(KeyEvent event, float distance_mm)
{
    switch (event) {
    case KEY_EVENT_PB15:
        AppMode_StartFollow();
        break;

    case KEY_EVENT_PB16:
        AppMode_StartDistanceFollow(distance_mm);
        break;

    case KEY_EVENT_PB2:
        AppMode_StartSlowFollow(distance_mm);
        break;

    case KEY_EVENT_NONE:
    default:
        break;
    }
}

void AppMode_Update(float distance_mm, uint32_t running_time_ms)
{
    if (s_mode == APP_MODE_FOLLOW_STOP_LINE) {
        /* 连续 30ms 检测到相邻 4 个黑线后直接断 PWM。 */
        if (Sensor_Stop_Line_Detected()) {
            flag = 2;
            Control_Stop();
            s_mode = APP_MODE_FINISHED;
        } else if (flag == 2) {
            /* Control() 内的安全停车判断同样会进入完成状态。 */
            s_mode = APP_MODE_FINISHED;
        }
    } else if (s_mode == APP_MODE_FOLLOW_CONTINUOUS) {
        Control_SetTrackDistance(distance_mm - s_distance_start_mm);
        /* 首次连续 4 路黑线仅锁存成绩；不修改 flag，因此小车不停。 */
        if (!s_stop_time_recorded && Sensor_Stop_Line_Detected()) {
            s_stop_time_ms = running_time_ms;
            s_stop_time_recorded = true;
        }
    } else if (s_mode == APP_MODE_FOLLOW_DISTANCE) {
        if ((distance_mm - s_distance_start_mm) >= FOLLOW_DISTANCE_MM) {
            flag = 2;
            Control_Stop();
            s_mode = APP_MODE_FINISHED;
        } else if (flag == 2) {
            s_mode = APP_MODE_FINISHED;
        }
    }
}

AppMode AppMode_Get(void)
{
    return s_mode;
}

bool AppMode_UsesDistanceScreen(void)
{
    return s_show_distance &&
           ((s_mode == APP_MODE_FOLLOW_DISTANCE) ||
            (s_mode == APP_MODE_FINISHED));
}

bool AppMode_UsesTimeScreen(void)
{
    return !AppMode_UsesDistanceScreen() &&
           ((s_mode == APP_MODE_FOLLOW_STOP_LINE) ||
            (s_mode == APP_MODE_FOLLOW_CONTINUOUS) ||
            (s_mode == APP_MODE_FINISHED));
}

float AppMode_GetDistanceProgress(float distance_mm)
{
    float progress = distance_mm - s_distance_start_mm;

    return (progress > 0.0f) ? progress : 0.0f;
}

uint32_t AppMode_GetDisplayTime(uint32_t running_time_ms)
{
    return s_stop_time_recorded ? s_stop_time_ms : running_time_ms;
}
