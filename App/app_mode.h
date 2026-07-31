#ifndef APP_APP_MODE_H
#define APP_APP_MODE_H

#include <stdbool.h>
#include <stdint.h>

#include "Key/key.h"

typedef enum {
    APP_MODE_MENU = 0,
    APP_MODE_FOLLOW_STOP_LINE,
    APP_MODE_FOLLOW_CONTINUOUS,
    APP_MODE_FOLLOW_DISTANCE,
    APP_MODE_FINISHED
} AppMode;

void AppMode_Init(void);
void AppMode_HandleKey(KeyEvent event, float distance_mm);
void AppMode_Update(float distance_mm, uint32_t running_time_ms);
AppMode AppMode_Get(void);
bool AppMode_UsesDistanceScreen(void);
bool AppMode_UsesTimeScreen(void);
float AppMode_GetDistanceProgress(float distance_mm);
uint32_t AppMode_GetDisplayTime(uint32_t running_time_ms);

#endif
