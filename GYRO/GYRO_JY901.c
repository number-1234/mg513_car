/**
 * @file    GYRO_JY901.c
 * @brief   JY901S 陀螺仪驱动（替代 MPU6050+DMP）
 *
 * API 与原 GYRO.c 完全一致。底层通过 WIT SDK（I2C 协议）
 * 与 JY901S 通信，复用 bsp_mpu6050 的硬件 I2C。
 *
 * JY901S 自带姿态解算，I2C 地址 0x50（7-bit）。
 * 角度寄存器 Roll(0x3d)、Pitch(0x3e)、Yaw(0x3f)。
 * 寄存器值 int16_t 映射到 ±180°（公式：reg / 32768 * 180）。
 */
#include "GYRO_JY901.h"

#include "jy901_i2c.h"
#include "wit_c_sdk.h"
#include "sys/sys.h"       /* delay_ms, system_millis */

/*
 * REG.h 定义了 Pitch=0x3e, Roll=0x3d, Yaw=0x3f 三个宏（寄存器地址），
 * 与全局变量名冲突。先用枚举保存寄存器索引，再 undef 宏。
 */
#include "REG.h"
enum {
    JY901_REG_ROLL  = Roll,   /* 0x3d */
    JY901_REG_PITCH = Pitch,  /* 0x3e */
    JY901_REG_YAW   = Yaw     /* 0x3f */
};
#undef Pitch
#undef Roll
#undef Yaw

/* ── 全局姿态角 ── */
float Pitch;
float Roll;
float Yaw;

/* ── 内部状态 ── */
static bool s_gyro_ready;
static volatile bool s_data_updated;  /* 回调标志：WIT SDK 成功读取到数据 */

/* WIT SDK 数据就绪回调 — I2C 读完成时被 WitReadReg 内部调用 */
static void JY901_DataCallback(uint32_t uiReg, uint32_t uiRegNum)
{
    (void)uiReg;
    (void)uiRegNum;
    s_data_updated = true;   /* 标记有新数据 */
}

/* delay_ms 适配：WIT SDK 要求 void(*)(uint16_t)，sys/delay 提供 void(*)(uint32_t) */
static void JY901_DelayMs(uint16_t ms)
{
    delay_ms((uint32_t)ms);
}

/* ── 扫描 JY901S ── */
static bool JY901_ScanSensor(void)
{
    int i, iRetry;

    /* JY901S 默认 I2C 地址 0x50（7-bit），扫描 0x01~0x7F */
    for (i = 1; i < 0x7F; i++) {
        WitInit(WIT_PROTOCOL_I2C, (uint8_t)i);
        iRetry = 2;
        do {
            s_data_updated = false;
            WitReadReg(AX, 3);        /* 读加速度寄存器验证设备存在 */
            delay_ms(5);
            if (s_data_updated) {     /* 回调被触发 = 读到有效数据 */
                return true;
            }
            iRetry--;
        } while (iRetry);
    }
    return false;
}
int16_t Yaw_zero=0;
/* ================================================================
 *  GYRO_Init — 初始化 JY901S
 *  返回 0=成功, 非0=失败
 * ================================================================ */
uint8_t GYRO_Init(void)
{
    Pitch = 0.0f;
    Roll  = 0.0f;
    Yaw   = 0.0f;
    s_gyro_ready = false;

    /* I2C 硬件已在 bsp_mpu6050/SYSCFG_DL_init 中初始化 */

    /* 注册 WIT SDK 回调 */
    WitI2cFuncRegister(JY901_I2cWrite, JY901_I2cRead);
    WitRegisterCallBack(JY901_DataCallback);
    WitDelayMsRegister(JY901_DelayMs);

    /* 扫描 JY901S */
    if (!JY901_ScanSensor()) {
        return 1;   /* 未找到传感器 */
    }

    s_gyro_ready = true;
     WitReadReg(JY901_REG_ROLL, 3);
     Yaw_zero   = (float)sReg[JY901_REG_YAW]   / 32768.0f * 180.0f;
    return 0;
}


/* ================================================================
 *  GYRO_Update — 读取最新姿态角
 *  返回 true=数据已更新, false=传感器未就绪或读取失败
 * ================================================================ */
bool GYRO_Update(void)
{
    if (!s_gyro_ready) {
        return false;
    }

    /* 读取 Roll/Pitch/Yaw（3 个连续寄存器，从 Roll=0x3d 开始） */
    s_data_updated = false;
    WitReadReg(JY901_REG_ROLL, 3);
    if (!s_data_updated) {
        return false;
    }

    /* sReg 是 int16_t，映射 0~32768 → 0~180° */
    /* Yaw 的 JY901S 输出范围为 0~360°，需转换到 ±180° */
    Pitch = (float)sReg[JY901_REG_PITCH] / 32768.0f * 180.0f;
    Roll  = (float)sReg[JY901_REG_ROLL]  / 32768.0f * 180.0f;
    Yaw   = ((float)sReg[JY901_REG_YAW]   / 32768.0f * 180.0f)-Yaw_zero;

    /* JY901S Yaw 输出 0~360°，控制模块期望 ±180° */
    if (Yaw > 180.0f) {
        Yaw -= 360.0f;
    }

    return true;
}

/* ================================================================
 *  GYRO_Is_Ready — 传感器是否就绪
 * ================================================================ */
bool GYRO_Is_Ready(void)
{
    return s_gyro_ready;
}
