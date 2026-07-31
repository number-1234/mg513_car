#include "imu_driver.h"

volatile IMU_Data_t g_imu_data;

/**
 * @brief 驱动初始化（开启 UART2 中断）
 */
void IMU_Init(void) {
    // 清除并使能 UART2 中断
    NVIC_ClearPendingIRQ(UART2_INT_IRQn);
    NVIC_EnableIRQ(UART2_INT_IRQn);
}

/**
 * @brief 串口单字节解算状态机
 */
static void IMU_ParseByte(uint8_t byte) {
    static uint8_t rx_buf[11];
    static uint8_t rx_cnt = 0;

    // 1. 找帧头 0x5A
    if (rx_cnt == 0) {
        if (byte == 0x5A) {
            rx_buf[rx_cnt++] = byte;
        }
        return;
    }

    // 2. 顺序接收后续字节
    rx_buf[rx_cnt++] = byte;

    // 3. 满 11 字节后校验并解算
    if (rx_cnt == 11) {
        rx_cnt = 0; // 重置计数器

        // 校验和运算
        uint8_t sum = 0;
        for (int i = 0; i < 10; i++) {
            sum += rx_buf[i];
        }

        if (sum != rx_buf[10]) {
            return; // 校验失败，直接丢弃
        }

        // 高低字节拼接 (需先强转为有符号 short 处理负数)
        short raw1 = (short)(((short)rx_buf[3] << 8) | rx_buf[2]);
        short raw2 = (short)(((short)rx_buf[5] << 8) | rx_buf[4]);
        short raw3 = (short)(((short)rx_buf[7] << 8) | rx_buf[6]);
        short raw4 = (short)(((short)rx_buf[9] << 8) | rx_buf[8]);

        uint8_t type = rx_buf[1];
        switch (type) {
            case 0xCC: // 加速度
                g_imu_data.acc[0] = (float)raw1 / 32768.0f * 16.0f;
                g_imu_data.acc[1] = (float)raw2 / 32768.0f * 16.0f;
                g_imu_data.acc[2] = (float)raw3 / 32768.0f * 16.0f;
                break;

            case 0xAA: // 角速度
                g_imu_data.gyro[0] = (float)raw1 / 32768.0f * 2000.0f;
                g_imu_data.gyro[1] = (float)raw2 / 32768.0f * 2000.0f;
                g_imu_data.gyro[2] = (float)raw3 / 32768.0f * 2000.0f;
                break;

            case 0xBB: // 角度 (Roll, Pitch, Yaw)
                g_imu_data.angle[0] = (float)raw1 / 32768.0f * 180.0f;
                g_imu_data.angle[1] = (float)raw2 / 32768.0f * 180.0f;
                g_imu_data.angle[2] = (float)raw3 / 32768.0f * 180.0f;
                break;

            case 0xDD: // 四元数
                g_imu_data.quat[0] = (float)raw1 / 32768.0f;
                g_imu_data.quat[1] = (float)raw2 / 32768.0f;
                g_imu_data.quat[2] = (float)raw3 / 32768.0f;
                g_imu_data.quat[3] = (float)raw4 / 32768.0f;
                break;

            default:
                break;
        }
    }
}

/**
 * @brief MSPM0G3507 UART2 中断服务函数
 */
void UART2_IRQHandler(void) {
    // 读取当前产生的中断类型
    switch (DL_UART_Main_getPendingInterrupt(UART2_INST)) {
        case DL_UART_MAIN_IIDX_RX: // 接收中断
            while (!DL_UART_Main_isRXFIFOEmpty(UART2_INST)) {
                uint8_t rx_data = DL_UART_Main_receiveData(UART2_INST);
                IMU_ParseByte(rx_data);
            }
            break;
        default:
            break;
    }
}

/**
 * @brief 给 IMU 模块写寄存器指令
 */
void IMU_SendCmd(uint8_t addr, uint16_t data) {
    uint8_t cmd[5];
    cmd[0] = 0x55;
    cmd[1] = 0xAA;
    cmd[2] = addr;
    cmd[3] = (uint8_t)(data & 0xFF);         // 低字节
    cmd[4] = (uint8_t)((data >> 8) & 0xFF);  // 高字节

    for (int i = 0; i < 5; i++) {
        DL_UART_Main_transmitDataBlocking(UART2_INST, cmd[i]);
    }
}

/**
 * @brief Z轴角度归零
 */
void IMU_SetYawZero(void) {
    IMU_SendCmd(0x13, 0x8E5F); // 1. 解锁
    delay_cycles(3200000);     // 延时约 100ms (基于 32MHz 主频)
    
    IMU_SendCmd(0x0A, 0x0004); // 2. 归零指令
    delay_cycles(3200000);
    
    IMU_SendCmd(0x00, 0x0000); // 3. 保存配置
}
