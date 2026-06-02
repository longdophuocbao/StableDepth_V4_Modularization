#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "Constants.h"

struct __attribute__((packed)) SerialTelemetry
{
    uint8_t head1 = 0xAA;
    uint8_t head2 = 0x55;
    float values[38];
    uint8_t checksum;
};

struct __attribute__((packed)) SerialCommand
{
    uint8_t head1;
    uint8_t head2;
    float values[19];
    uint8_t checksum;
};

uint8_t calculateChecksum(uint8_t *data, size_t len);

#ifdef DEBUG_SERIAL
void serialTuningTask(void *param);
#endif

#endif // TELEMETRY_H
