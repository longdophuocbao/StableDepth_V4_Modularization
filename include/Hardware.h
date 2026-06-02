#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

void driveActuator(float pwm_val);
float readLiftingSensorRaw();
float readTailboardSensorRaw();
void initHardware();

#endif // HARDWARE_H
