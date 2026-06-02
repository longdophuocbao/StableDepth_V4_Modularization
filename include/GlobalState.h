#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <ESP32Servo.h>
#include "Models.h"
#include "Filters.h"

extern SemaphoreHandle_t g_mutex;

extern Adafruit_ADS1115 liftingsensor;
extern Adafruit_ADS1115 tailboardsensor;
extern ESP32PWM pwmIN1;
extern ESP32PWM pwmIN2;

extern volatile float g_liftingangle;
extern volatile float g_tailboardangle;
extern volatile float g_lifting_filtered;
extern volatile float g_tail_filtered;
extern volatile float g_depth_target;
extern volatile float g_setpoint;
extern volatile int g_mode;
extern volatile bool g_run_state;
extern volatile float g_alpha_manual;
extern volatile float g_sp_amp;
extern volatile float g_sp_freq;
extern volatile float g_sp_raw;

extern volatile float g_e;
extern volatile float g_de;
extern volatile float g_s;
extern volatile float g_u;
extern volatile float g_u_direct;
extern volatile float g_u_eq;
extern volatile float g_u_sw;
extern volatile float g_u_sw_i;
extern volatile float g_e_int;
extern volatile float g_estimate_depth;
extern volatile float g_y_pred;
extern volatile float g_dot_alpha_actual_raw;
extern volatile float g_dot_alpha_ref;
extern volatile float g_ddot_alpha_ref;

extern volatile float g_Kp;
extern volatile float g_Ki;
extern volatile float g_Kd;
extern volatile float g_K1;
extern volatile float g_K2;

extern volatile float g_K;
extern volatile float g_tau1;
extern volatile float g_L;
extern volatile float g_lift_offset;
extern volatile float g_tail_offset;

extern volatile float g_fc_lifting;
extern volatile float g_fc_tailboard;
extern volatile float g_omega_ref;
extern volatile float g_fc_de;

extern volatile bool g_model_dirty;

extern volatile float g_lifting_raw_val;
extern volatile float g_tail_raw_val;
extern volatile float g_sp_raw_val;

extern SOIPDTModel g_model_nd;
extern SOIPDTModel g_model_wd;

extern ReferenceFilter refFilter;
extern LPF2ndOrder filterLifting;
extern LPF2ndOrder filterTailboard;
extern MedianFilter medianLifting;
extern MedianFilter medianTailboard;
extern LPF2ndOrder filterAlpha_actual_dot;

#endif // GLOBAL_STATE_H
