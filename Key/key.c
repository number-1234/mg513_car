#include <stdbool.h>
#include <stdint.h>

#include "key.h"
#include "sys/sys.h"

#define KEY_DEBOUNCE_MS 30U
#define KEY_PORT GPIOB
#define KEY_PB15_PIN DL_GPIO_PIN_15
#define KEY_PB16_PIN DL_GPIO_PIN_16
#define KEY_PB2_PIN  DL_GPIO_PIN_2
#define KEY_ALL_PINS (KEY_PB15_PIN | KEY_PB16_PIN | KEY_PB2_PIN)

static bool s_pb15_latched;
static bool s_pb16_latched;
static bool s_pb2_latched;
static uint32_t s_last_event_ms;

void Key_Init(void)
{
    /* PB15/PB16/PB2 的输入上拉由 SysConfig 在 SYSCFG_DL_init() 中完成。 */
    DL_GPIO_disableOutput(KEY_PORT, KEY_ALL_PINS);

    s_pb15_latched = false;
    s_pb16_latched = false;
    s_pb2_latched = false;
    s_last_event_ms = system_millis() - KEY_DEBOUNCE_MS;
}

KeyEvent Key_GetPressed(void)
{
    uint32_t pins = DL_GPIO_readPins(KEY_PORT, KEY_ALL_PINS);
    bool pb15_pressed = ((pins & KEY_PB15_PIN) == 0U);
    bool pb16_pressed = ((pins & KEY_PB16_PIN) == 0U);
    bool pb2_pressed = ((pins & KEY_PB2_PIN) == 0U);
    uint32_t now_ms = system_millis();

    if (!pb15_pressed) s_pb15_latched = false;
    if (!pb16_pressed) s_pb16_latched = false;
    if (!pb2_pressed) s_pb2_latched = false;

    if ((uint32_t)(now_ms - s_last_event_ms) < KEY_DEBOUNCE_MS) {
        return KEY_EVENT_NONE;
    }

    if (pb15_pressed && !s_pb15_latched) {
        s_pb15_latched = true;
        s_last_event_ms = now_ms;
        return KEY_EVENT_PB15;
    }
    if (pb16_pressed && !s_pb16_latched) {
        s_pb16_latched = true;
        s_last_event_ms = now_ms;
        return KEY_EVENT_PB16;
    }
    if (pb2_pressed && !s_pb2_latched) {
        s_pb2_latched = true;
        s_last_event_ms = now_ms;
        return KEY_EVENT_PB2;
    }

    return KEY_EVENT_NONE;
}
