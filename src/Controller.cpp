#include "Controller.h"
#include "GlobalState.h"
#include "Constants.h"
#include "Hardware.h"
#include <math.h>

const float C_B2 = -0.03521f, C_B1 = 5.36830f, C_B0 = -48.1255f;
const float C_D2 = -0.03691f, C_D1 = 9.84580f, C_D0 = -75.4150f;
const float GAIN_DD_DB = 1.7314f; // d(depth)/d(beta)  mm/mm
const float GAIN_DA_DD = 0.1172f; // d(alpha)/d(depth) °/mm

float get_alpha_ref(float alpha, float beta_actual, float depth_target)
{
    float beta_pred = C_B2 * alpha * alpha + C_B1 * alpha + C_B0;
    float delta_beta = beta_actual - beta_pred; // terrain signal
    float depth_nom = C_D2 * alpha * alpha + C_D1 * alpha + C_D0;
    float depth_est = depth_nom + GAIN_DD_DB * delta_beta; // compensated depth
    float depth_err = depth_target - depth_est;
    float alpha_ref = alpha + GAIN_DA_DD * depth_err; // incremental correction
    return fmaxf(11.0f, fminf(30.0f, alpha_ref));
}

double calculate_estimate_depth(double liftingange, double tailboardangle)
{
    float estimate_depth = 0.0;
    if (tailboardangle < 0.5)
        return 0.0;
    estimate_depth = liftingange * 1.1 + tailboardangle * 2.0;
    return (double)estimate_depth;
}

void runController()
{
    float liftingangle = g_lifting_filtered;
    float tailangle = g_tail_filtered;
    float depth_target, Kp, Ki, Kd, K1, K2, K, tau1, L, alpha_manual, sp_amp, sp_freq, u_direct;
    int mode;
    bool run_state, dirty = false;

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        depth_target = g_depth_target;
        Kp = g_Kp;
        Ki = g_Ki;
        Kd = g_Kd;
        K1 = g_K1;
        K2 = g_K2;
        K = g_K;
        tau1 = g_tau1;
        L = g_L;
        mode = g_mode;
        run_state = g_run_state;
        alpha_manual = g_alpha_manual;
        sp_amp = g_sp_amp;
        sp_freq = g_sp_freq;
        u_direct = g_u_direct;
        dirty = g_model_dirty;
        g_model_dirty = false;
        xSemaphoreGive(g_mutex);
    }
    else
        return;

    // --- MODE HANDLING ---
    float sp_raw = 20.0f;
    if (mode == 0)
    { // Auto
        sp_raw = (float)get_alpha_ref(liftingangle, tailangle, depth_target);
    }
    else if (mode == 1)
    { // Manual
        sp_raw = alpha_manual;
    }
    else if (mode == 2)
    { // Step Wave
        float period = 1.0f / (sp_freq > 0.001f ? sp_freq : 0.001f);
        float time_in_period = fmodf(millis() / 1000.0f, period);
        sp_raw = alpha_manual + (time_in_period < period / 2.0f ? sp_amp : -sp_amp);
    }
    else if (mode == 3)
    { // Oscillation
        sp_raw = alpha_manual + sp_amp * sinf(2.0f * M_PI * sp_freq * (millis() / 1000.0f));
    }
    else if (mode == 4)
    { // Direct U
        sp_raw = liftingangle;
    }
    sp_raw = constrain(sp_raw, 0.0f, 40.0f);
    g_sp_raw_val = sp_raw;

    static float u_sw_int = 0.0f;
    static float u_prev = 0.0f;
    static float y_pred_prev = 0.0f;
    static int prev_mode = -1;
    static bool prev_run_state = false;
    static bool first_run = true;

    if (first_run || mode != prev_mode || (run_state && !prev_run_state))
    {
        refFilter.x1 = liftingangle;
        refFilter.x2 = 0.0f;
        g_e_int = 0.0f;
        u_sw_int = 0.0f;
        y_pred_prev = liftingangle;
        first_run = false;
    }
    prev_mode = mode;
    prev_run_state = run_state;

    if (!run_state)
    {
        g_e_int = 0.0f;
        u_sw_int = 0.0f;
        refFilter.x1 = liftingangle;
        refFilter.x2 = 0.0f;
        g_model_nd.reset();
        g_model_wd.reset();
        y_pred_prev = liftingangle;
    }

    ReferenceFilter::FilterOutput refOut = refFilter.update(sp_raw, DT);
    float alpha_ref = refOut.val, dot_alpha_ref = refOut.dot, ddot_alpha_ref = refOut.ddot;

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        g_setpoint = alpha_ref;
        g_dot_alpha_ref = dot_alpha_ref;
        g_ddot_alpha_ref = ddot_alpha_ref;
        g_tailboardangle = tailangle;
        xSemaphoreGive(g_mutex);
    }

    if (dirty)
    {
        g_model_wd.K = -K;
        g_model_wd.tau1 = tau1;
        g_model_wd.L = L;
        g_model_wd.init(DT);
        g_model_nd.K = -K;
        g_model_nd.tau1 = tau1;
        g_model_nd.L = 0.0f;
        g_model_nd.init(DT);
    }

    float y_m = g_model_nd.step(u_prev, DT);
    float y_m_d = g_model_wd.step(u_prev, DT);
    float y_pred = liftingangle + (y_m - y_m_d);

    float delta_y = y_pred - y_pred_prev;

    if (fabsf(delta_y) < 0.02f)
    {
        delta_y = 0.0f;
    }
    else
    {
        y_pred_prev = y_pred;
    }

    float alpha_actual_dot_raw = delta_y / DT;
    g_dot_alpha_actual_raw = alpha_actual_dot_raw;
    float alpha_actual_dot = filterAlpha_actual_dot.update(alpha_actual_dot_raw);

    float e = y_pred - alpha_ref, de = alpha_actual_dot - dot_alpha_ref;

    e = constrain(e, -40.0f, 40.0f);
    g_e_int = constrain(g_e_int + e * DT, -2.0f, 2.0f);

    float s = Kp * e + Ki * g_e_int + Kd * de;

    float safe_tau1 = (fabsf(tau1) < 1e-4f) ? 1e-4f : tau1;
    float a1 = 1.0f / safe_tau1, a2 = 0.0f, b = (-K) / safe_tau1;

    float u_eq = 0.0f;
    float safe_Kd = (fabsf(Kd) < 1e-4f) ? ((Kd >= 0.0f) ? 1e-4f : -1e-4f) : Kd;
    if (fabsf(b) > 1e-6f)
    {
        u_eq = (1.0f / b) * (a1 * alpha_actual_dot + a2 * y_pred + ddot_alpha_ref - (Kp / safe_Kd) * de - (Ki / safe_Kd) * e);
    }
    u_eq = constrain(u_eq, -PWM_MAX_ABS, PWM_MAX_ABS);

    float sign_s = (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
    u_sw_int = constrain(u_sw_int - K2 * sign_s * DT, -2.0f, 2.0f);

    float u_sw = 0.0f;
    float b_Kd = b * safe_Kd;
    if (fabsf(b_Kd) > 1e-6f)
    {
        u_sw = (1.0f / b_Kd) * (-K1 * sqrtf(fabsf(s)) * sign_s + u_sw_int);
    }
    u_sw = constrain(u_sw, -PWM_MAX_ABS, PWM_MAX_ABS);

    const float PWM_DEADBAND = 0.1f;
    float u_out = constrain(u_eq + u_sw, -PWM_MAX_ABS, PWM_MAX_ABS);

    float u_out_hardware = 0.0f;
    if (fabsf(u_out) > 0.05f)
    {
        if (u_out > 0.0f)
            u_out_hardware = u_out + PWM_DEADBAND;
        else
            u_out_hardware = u_out - PWM_DEADBAND;
    }
    
    u_out_hardware = constrain(u_out_hardware, -PWM_MAX_ABS, PWM_MAX_ABS);
    if (run_state) {
        driveActuator(u_out_hardware);
        u_prev = u_out;
    } else {
        driveActuator(0.0f);
        u_prev = 0.0f;
    }

    float estimatedepth = (float)calculate_estimate_depth(liftingangle, tailangle);
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        g_liftingangle = liftingangle;
        g_e = e;
        g_de = de;
        g_s = s;
        g_u = u_out;
        g_u_eq = u_eq;
        g_u_sw = u_sw;
        g_u_sw_i = u_sw_int;
        g_estimate_depth = estimatedepth;
        g_y_pred = y_pred;
        g_dot_alpha_actual_raw = alpha_actual_dot_raw;
        g_dot_alpha_ref = dot_alpha_ref;
        g_ddot_alpha_ref = ddot_alpha_ref;
        xSemaphoreGive(g_mutex);
    }
}

void sensorReadTask(void *param)
{
    TickType_t xLastWake = xTaskGetTickCount();
    while (true)
    {
        float lifting_raw = 266.6f - readLiftingSensorRaw();
        lifting_raw = constrain(lifting_raw, 0, 40);
        float tail_raw = readTailboardSensorRaw() - g_tail_offset;

        tail_raw = constrain(tail_raw, 0, 80);

        float lifting_median = constrain(medianLifting.update(lifting_raw), 0.0f, 40.0f);
        float tail_median = constrain(medianTailboard.update(tail_raw), 0.0f, 80.0f);

        g_lifting_filtered = constrain(filterLifting.update(lifting_median), 0.0f, 40.0f);
        g_tail_filtered = constrain(filterTailboard.update(tail_median), 0.0f, 80.0f);
        g_lifting_raw_val = lifting_raw;
        g_tail_raw_val = tail_raw;

        vTaskDelayUntil(&xLastWake, SENSOR_DT_TICKS);
    }
}

void controlTask(void *param)
{
    TickType_t xLastWake = xTaskGetTickCount();
    while (true)
    {
        runController();
        vTaskDelayUntil(&xLastWake, DT_TICKS);
    }
}
