#ifndef FILTERS_H
#define FILTERS_H

#include <Arduino.h>
#include <math.h>

struct ReferenceFilter
{
    float x1, x2;
    float omega;
    float zeta;

    void init(float _omega, float _zeta)
    {
        omega = _omega;
        zeta = _zeta;
        x1 = x2 = 0.0f;
    }

    void reinit(float _omega, float _zeta)
    {
        omega = _omega;
        zeta = _zeta;
    }

    struct FilterOutput
    {
        float val, dot, ddot;
    };

    FilterOutput update(float r, float dt)
    {
        float error = r - x1;
        float x_ddot = (omega * omega) * error - (2.0f * zeta * omega) * x2;
        x2 += x_ddot * dt;
        x1 += x2 * dt;
        return {x1, x2, x_ddot};
    }
};

struct MedianFilter
{
    static const int N = 7;
    float buf[N];
    int idx;
    float last_out;
    float threshold;
    int outlier_count;

    void init(float initial_val, float _threshold = 2.0f)
    {
        idx = 0;
        outlier_count = 0;
        last_out = initial_val;
        threshold = _threshold;
        for (int i = 0; i < N; i++)
            buf[i] = initial_val;
    }

    float update(float x)
    {
        if (fabsf(x - last_out) > threshold)
        {
            outlier_count++;

            // Nếu lệch liên tục 5 lần (ví dụ), ép reset về giá trị mới luôn
            if (outlier_count > 5)
            {
                last_out = x;
                for (int i = 0; i < N; i++)
                    buf[i] = x;
                outlier_count = 0;
            }
            else
            {
                // Vẫn chặn như cũ nếu số lần vượt ngưỡng còn ít (để chống nhiễu ngắn hạn)
                x = last_out + copysignf(threshold * 0.5f, x - last_out);
            }
        }
        else
        {
            outlier_count = 0; // Trở lại bình thường thì reset bộ đếm
        }

        buf[idx] = x;
        idx = (idx + 1) % N;

        float tmp[N];
        for (int i = 0; i < N; i++)
            tmp[i] = buf[i];
        for (int i = 1; i < N; i++)
        {
            float key = tmp[i];
            int j = i - 1;
            while (j >= 0 && tmp[j] > key)
            {
                tmp[j + 1] = tmp[j];
                j--;
            }
            tmp[j + 1] = key;
        }

        last_out = tmp[N / 2];
        return last_out;
    }
};

struct LPF2ndOrder
{
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;

    void init(float fc, float fs)
    {
        computeCoeffs(fc, fs);
        x1 = x2 = y1 = y2 = 0.0f;
    }

    void reinit(float fc, float fs)
    {
        computeCoeffs(fc, fs);
    }

    void computeCoeffs(float fc, float fs)
    {
        float w0 = 2.0f * M_PI * fc / fs;
        float cosw0 = cosf(w0);
        float alpha = sinf(w0) / 1.4142f;
        float a0 = 1.0f + alpha;
        b0 = ((1.0f - cosw0) / 2.0f) / a0;
        b1 = (1.0f - cosw0) / a0;
        b2 = ((1.0f - cosw0) / 2.0f) / a0;
        a1 = (-2.0f * cosw0) / a0;
        a2 = (1.0f - alpha) / a0;
    }

    float update(float x)
    {
        float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        return y;
    }
};

#endif // FILTERS_H
