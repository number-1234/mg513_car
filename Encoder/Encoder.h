/**
 * @file    Encoder.h
 * @brief   双路编码器模块 — 脉冲计数 + 轮速换算
 *
 * 编码器 A（左电机）：PA17(AA, 中断) + PA16(AB, 方向)
 * 编码器 B（右电机）：PA15(BA, 中断) + PA14(BB, 方向)
 *
 * 参数：电机轴 260PPR × 减速比 28:1 = 轮轴 7280 脉冲/圈，轮径 65mm，采样 100ms
 */
#ifndef ENCODER_ENCODER_H
#define ENCODER_ENCODER_H

#include <stdint.h>

/* 实时脉冲计数（GPIO 中断累加，Read_Encoder 清零） */
extern volatile int32_t EncoderA_CNT;
extern volatile int32_t EncoderB_CNT;

/* 线速度 mm/s（绝对值，≥0） */
extern volatile float   EncoderA_VEL;
extern volatile float   EncoderB_VEL;

/* 最近一次清零前的原始计数（含符号，调试用） */
extern volatile int32_t EncoderA_Last_Raw;
extern volatile int32_t EncoderB_Last_Raw;

void Encoder_Init(void);
void Encoder_Reset(void);
void Encodering(void);     /* GPIO 中断回调，main.c → GROUP1_IRQHandler */
void Read_Encoder(void);   /* 100ms 定时器回调，换算速度 */

#endif
