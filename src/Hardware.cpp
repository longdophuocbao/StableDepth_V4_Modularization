#include "Hardware.h"
#include "GlobalState.h"
#include "Constants.h"

void initHardware()
{
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    if (liftingsensor.begin(0x48))
    {
        liftingsensor.setGain(GAIN_ONE);
        liftingsensor.setDataRate(RATE_ADS1115_860SPS);
        liftingsensor.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);
    }
    if (tailboardsensor.begin(0x49))
    {
        tailboardsensor.setGain(GAIN_ONE);
        tailboardsensor.setDataRate(RATE_ADS1115_860SPS);
        tailboardsensor.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);
    }
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    pwmIN1.attachPin(PIN_MOTOR_IN1, PWM_FREQ_HZ, PWM_BITS);
    pwmIN2.attachPin(PIN_MOTOR_IN2, PWM_FREQ_HZ, PWM_BITS);
}

void driveActuator(float pwm_val)
{
    pwm_val = constrain(pwm_val, -PWM_MAX_ABS, PWM_MAX_ABS);
    if (pwm_val > 0.0f)
    {
        pwmIN1.writeScaled(pwm_val);
        pwmIN2.writeScaled(0.0f);
    }
    else if (pwm_val < 0.0f)
    {
        pwmIN1.writeScaled(0.0f);
        pwmIN2.writeScaled(abs(pwm_val));
    }
    else
    {
        pwmIN1.writeScaled(0.0f);
        pwmIN2.writeScaled(0.0f);
    }
}

float readLiftingSensorRaw()
{
    int16_t adc = liftingsensor.getLastConversionResults();
    float voltage = liftingsensor.computeVolts(adc);
    return (84.22851f * voltage);
}

float readTailboardSensorRaw()
{
    int16_t adc = tailboardsensor.getLastConversionResults();
    float voltage = tailboardsensor.computeVolts(adc);
    return (61.0351f * voltage);
}
