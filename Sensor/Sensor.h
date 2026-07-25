#ifndef SENSOR_SENSOR_H
#define SENSOR_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

extern volatile uint8_t Sensor_Bits;

void Sensor_Init(void);
void Sensor_Read(void);
bool Sensor_Has_Line(void);

void Follow_Route(void);
int Incremental_Quantity(void);
void Sensor_Calibration(void);

#endif
