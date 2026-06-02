#include "Telemetry.h"
#include "GlobalState.h"
#include "Constants.h"

uint8_t calculateChecksum(uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++)
        checksum ^= data[i];
    return checksum;
}

#ifdef DEBUG_SERIAL
void serialTuningTask(void *param)
{
    TickType_t xLastWake = xTaskGetTickCount();
    while (true)
    {
        SerialTelemetry msg;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            float *v = msg.values;
            v[0] = millis() / 1000.0f;
            v[1] = g_y_pred;
            v[2] = g_estimate_depth;
            v[3] = (float)g_mode;
            v[4] = g_lifting_raw_val;
            v[5] = g_liftingangle;
            v[6] = g_tail_raw_val;
            v[7] = g_tailboardangle;
            v[8] = g_dot_alpha_actual_raw;
            v[9] = g_depth_target;
            v[10] = g_setpoint;
            v[11] = g_dot_alpha_ref;
            v[12] = g_ddot_alpha_ref;
            v[13] = g_alpha_manual;
            v[14] = g_omega_ref;
            v[15] = g_e;
            v[16] = g_de;
            v[17] = g_s;
            v[18] = g_u;
            v[19] = g_u_eq;
            v[20] = g_u_sw;
            v[21] = g_u_sw_i;
            v[22] = g_Kp;
            v[23] = g_Ki;
            v[24] = g_Kd;
            v[25] = g_K1;
            v[26] = g_K2;
            v[27] = g_K;
            v[28] = g_tau1;
            v[29] = g_L;
            v[30] = g_fc_lifting;
            v[31] = g_fc_tailboard;
            v[32] = g_lift_offset;
            v[33] = g_tail_offset;
            v[34] = g_e_int;
            v[35] = g_fc_de;
            v[36] = g_sp_amp;
            v[37] = g_sp_freq;
            xSemaphoreGive(g_mutex);
        }
        msg.checksum = calculateChecksum((uint8_t *)&msg.values, sizeof(msg.values));
        Serial.write((uint8_t *)&msg, sizeof(msg));

        if (Serial.available() >= sizeof(SerialCommand))
        {
            uint8_t buf[sizeof(SerialCommand)];
            if (Serial.peek() == 0xAA)
            {
                Serial.readBytes(buf, sizeof(SerialCommand));
                SerialCommand *cmd = (SerialCommand *)buf;
                if (cmd->head1 == 0xAA && cmd->head2 == 0x55 && calculateChecksum((uint8_t *)&cmd->values, sizeof(cmd->values)) == cmd->checksum)
                {
                    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
                    {
                        float *cv = cmd->values;
                        g_run_state = (cv[0] > 0.5f);
                        g_mode = (int)cv[1];
                        g_depth_target = cv[2];
                        g_alpha_manual = cv[3];
                        g_sp_amp = cv[4];
                        g_sp_freq = cv[5];

                        g_Kp = cv[6];
                        g_Ki = cv[7];
                        g_Kd = cv[8];
                        g_K1 = cv[9];
                        g_K2 = cv[10];

                        g_K = cv[11];
                        g_tau1 = cv[12];
                        g_L = cv[13] / 1000.0f; // HTML sends L in ms

                        if (g_fc_lifting != cv[14] || g_fc_tailboard != cv[15] || g_fc_de != cv[16] || g_omega_ref != cv[17])
                        {
                            g_fc_lifting = cv[14];
                            g_fc_tailboard = cv[15];
                            g_fc_de = cv[16];
                            g_omega_ref = cv[17];

                            filterLifting.reinit(g_fc_lifting, 500.0f);
                            filterTailboard.reinit(g_fc_tailboard, 500.0f);
                            filterAlpha_actual_dot.reinit(g_fc_de, 500.0f);
                            refFilter.reinit(g_omega_ref, 1.0f);
                        }
                        
                        g_u_direct = cv[18];

                        g_model_dirty = true;
                        xSemaphoreGive(g_mutex);
                    }
                }
            }
            else
                Serial.read();
        }
        vTaskDelayUntil(&xLastWake, SERIALDEBUG_DT_TICKS);
    }
}
#endif
