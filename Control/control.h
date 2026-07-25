#ifndef CONTROL_CONTROL_H
#define CONTROL_CONTROL_H

extern volatile int Motor_Left;
extern volatile int Motor_Right;
extern volatile float Turn_Zero_Yaw;
extern volatile float Straight_Zero_Yaw;

/* PID 分量（PID 调参用） */
extern volatile float PID_A_Target;
extern volatile float PID_B_Target;
extern volatile float PID_A_Error;
extern volatile float PID_B_Error;
extern volatile float PID_A_P;
extern volatile float PID_A_I;
extern volatile float PID_B_P;
extern volatile float PID_B_I;

/* 循迹 PD 分量（调参用） */
extern volatile float Line_Deviation;
extern volatile float Line_Bias_P;
extern volatile float Line_Bias_D;
extern volatile float Line_Bias_Total;

void Control_Init(void);
void Control(void);
void Control_Stop(void);

void Set_Pwm(int Left, int Right);
float PWM_Limit(float value, float maximum, float minimum);
float PID_A(float Encoder, float Target);
float PID_B(float Encoder, float Target);
float GYRO_Control(float now, float target);

float Control_Get_Angle(void);
float Control_Get_Bias(void);

#endif
