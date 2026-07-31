#ifndef SENSOR_SENSOR_H
#define SENSOR_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

extern volatile uint8_t Sensor_Bits;

void Sensor_Init(void);
void Sensor_Read(void);
bool Sensor_Has_Line(void);
bool Sensor_Is_Online(void);
uint8_t Sensor_Black_Count(void);
void Sensor_Enable_Stop_Line_After(uint32_t delay_ms);
void Sensor_Disable_Stop_Line(void);
bool Sensor_Stop_Line_Enabled(void);
bool Sensor_Stop_Line_Detected(void);

void Follow_Route(void);
int Incremental_Quantity(void);
void Sensor_Calibration(void);

#endif
