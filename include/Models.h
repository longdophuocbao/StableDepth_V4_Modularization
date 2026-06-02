#ifndef MODELS_H
#define MODELS_H

#include <Arduino.h>
#include "Constants.h"

struct SOIPDTModel
{
    float K = 0.076976f * 100.0f;
    float tau1 = 1.32f;
    float L = 0.002f;

    float y_k1 = 0.0f;
    float y_k2 = 0.0f;

    float u_buf[MAX_DELAY_SAMPLES];
    int buf_head = 0;
    int d = 0;

    void init(float dt)
    {
        d = (int)roundf(L / dt);
        if (d >= MAX_DELAY_SAMPLES)
            d = MAX_DELAY_SAMPLES - 1;
        memset(u_buf, 0, sizeof(u_buf));
        y_k1 = y_k2 = 0.0f;
        buf_head = 0;
    }

    float step(float u_in, float dt)
    {
        u_buf[buf_head] = u_in;
        int idx = (buf_head - d + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
        float u_d = u_buf[idx];
        buf_head = (buf_head + 1) % MAX_DELAY_SAMPLES;

        float dt2 = dt * dt;
        float denom = tau1 + dt;
        float y_n = (K * dt2 * u_d + (2.0f * tau1 + dt) * y_k1 - tau1 * y_k2) / denom;

        y_k2 = y_k1;
        y_k1 = y_n;
        return y_n;
    }

    void reset()
    {
        y_k1 = y_k2 = 0.0f;
        buf_head = 0;
        memset(u_buf, 0, sizeof(u_buf));
    }
};

#endif // MODELS_H
