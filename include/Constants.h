#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// I2C cho ADS1115
#define PIN_SDA 26
#define PIN_SCL 25

// H-bridge: 2 kênh PWM độc lập
#define PIN_MOTOR_IN1 33 // chân PWM kênh thuận
#define PIN_MOTOR_IN2 32 // chân PWM kênh nghịch

#define PWM_FREQ_HZ 20000
#define PWM_BITS 10
#define PWM_MAX_ABS 0.95f

// CHU KỲ LẤY MẪU
static const float DT = 0.002f; // 2 ms = 500 Hz
static const TickType_t DT_TICKS = pdMS_TO_TICKS(2);
static const TickType_t SENSOR_DT_TICKS = pdMS_TO_TICKS(2);

#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
static const TickType_t SERIALDEBUG_DT_TICKS = pdMS_TO_TICKS(20); // 20ms = 50hz
#endif

static const int MAX_DELAY_SAMPLES = 2000;

#endif // CONSTANTS_H
