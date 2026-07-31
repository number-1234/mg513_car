#ifndef KEY_KEY_H
#define KEY_KEY_H

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PB15,
    KEY_EVENT_PB16,
    KEY_EVENT_PB2
} KeyEvent;

/* PB15/PB16/PB2 由 SysConfig 配置为内部上拉输入，按下时为低电平。 */
void Key_Init(void);
KeyEvent Key_GetPressed(void);

#endif
