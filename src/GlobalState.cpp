#include "GlobalState.h"
#include "Constants.h"

SemaphoreHandle_t g_mutex;

Adafruit_ADS1115 liftingsensor;
Adafruit_ADS1115 tailboardsensor;
ESP32PWM pwmIN1;
ESP32PWM pwmIN2;

volatile float g_liftingangle = 40.0f;
volatile float g_tailboardangle = 0.0f;
volatile float g_lifting_filtered = 40.0f;
volatile float g_tail_filtered = 0.0f;
volatile float g_depth_target = 100.0f;
volatile float g_setpoint = 20.0f;
volatile int g_mode = 1; // 0: Auto, 1: Manual, 2: Step, 3: Oscillation, 4: Direct U
volatile bool g_run_state = false;
volatile float g_alpha_manual = 20.0f;
volatile float g_sp_amp = 10.0f;
volatile float g_sp_freq = 0.1f;
volatile float g_sp_raw = 20.0f; // Raw target before ref filter

volatile float g_e = 0.0f;
volatile float g_de = 0.0f;
volatile float g_s = 0.0f;
volatile float g_u = 0.0f;
volatile float g_u_direct = 0.0f;
volatile float g_u_eq = 0.0f;
volatile float g_u_sw = 0.0f;
volatile float g_u_sw_i = 0.0f;
volatile float g_e_int = 0.0f;
volatile float g_estimate_depth = 0.0f;
volatile float g_y_pred = 0.0f;
volatile float g_dot_alpha_actual_raw = 0.0f;
volatile float g_dot_alpha_ref = 0.0f;
volatile float g_ddot_alpha_ref = 0.0f;

volatile float g_Kp = 1.9f;
volatile float g_Ki = 1.5f;
volatile float g_Kd = 1.33f;
volatile float g_K1 = 3.7f;
volatile float g_K2 = 0.5f;

volatile float g_K = 35.0f;
volatile float g_tau1 = 1.2244f;
volatile float g_L = 0.0331f;
volatile float g_lift_offset = 266.6f;
volatile float g_tail_offset = 126.43f;

volatile float g_fc_lifting = 10.0f;
volatile float g_fc_tailboard = 10.0f;
volatile float g_omega_ref = 8.0f;
volatile float g_fc_de = 10.0f;

volatile bool g_model_dirty = true;

volatile float g_lifting_raw_val = 0.0f;
volatile float g_tail_raw_val = 0.0f;
volatile float g_sp_raw_val = 0.0f;

SOIPDTModel g_model_nd;
SOIPDTModel g_model_wd;

ReferenceFilter refFilter;
LPF2ndOrder filterLifting;
LPF2ndOrder filterTailboard;
MedianFilter medianLifting;
MedianFilter medianTailboard;
LPF2ndOrder filterAlpha_actual_dot;
