#ifndef CONTROL_CONTROL_H
#define CONTROL_CONTROL_H

extern volatile int Motor_Left;
extern volatile int Motor_Right;
extern volatile float Turn_Zero_Yaw;
extern volatile float Straight_Zero_Yaw;

void Control_Init(void);
void Control(void);
void Control_Stop(void);

void  Set_Pwm(int Left, int Right);
float PWM_Limit(float value, float maximum, float minimum);
float PID_A(float Encoder, float Target);
float PID_B(float Encoder, float Target);
float GYRO_Control(float now, float target);
void beep_10ms();
float Control_Get_Angle(void);
float Control_Get_Bias(void);

#endif
