#include <Arduino.h>
#include "Constants.h"
#include "GlobalState.h"
#include "Hardware.h"
#include "Controller.h"
#include "Telemetry.h"

void setup()
{
    setCpuFrequencyMhz(240);
    delay(500);
    Serial.begin(250000);
    delay(500);
#ifdef DEBUG_SERIAL_2ESP
    Serial2.begin(250000, SERIAL_8N1, 16, 17);
    delay(500);
#endif
    Serial.println("\n=== Tractor SOIPDT + Super-Twisting SMC (Modular) ===");

    initHardware();
    Serial.println("Khởi tạo phần cứng thành công!");

    // Đợi một chút để ADS1115 hoàn thành lượt chuyển đổi đầu tiên
    delay(100);
    float init_lifting = 266.6f - readLiftingSensorRaw();
    init_lifting = constrain(init_lifting, 0.0f, 40.0f);
    float init_tail = readTailboardSensorRaw() - g_tail_offset;
    init_tail = constrain(init_tail, 0.0f, 80.0f);

    g_mutex = xSemaphoreCreateMutex();

    filterLifting.init(g_fc_lifting, 500.0f);
    filterTailboard.init(g_fc_tailboard, 500.0f);
    medianLifting.init(init_lifting, 1.0f);
    medianTailboard.init(init_tail, 1.0f);
    filterAlpha_actual_dot.init(g_fc_de, 500.0f);
    refFilter.init(g_omega_ref, 1.0f);

    g_model_wd.K = g_K;
    g_model_wd.tau1 = g_tau1;
    g_model_wd.L = g_L;
    g_model_wd.init(DT);

    g_model_nd.K = g_K;
    g_model_nd.tau1 = g_tau1;
    g_model_nd.L = 0.0f;
    g_model_nd.init(DT);

    g_model_dirty = false;

    xTaskCreatePinnedToCore(sensorReadTask, "SensRead", 4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(controlTask, "SMC_Ctrl", 8192, nullptr, 2, nullptr, 1);
#ifdef DEBUG_SERIAL
    xTaskCreatePinnedToCore(serialTuningTask, "SerTune", 4096, nullptr, 1, nullptr, 0);
#endif

    Serial.println("=== Ready! Serial Tuning Active ===\n");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(100));
}
