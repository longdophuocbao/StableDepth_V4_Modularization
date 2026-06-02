#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

void runController();
void sensorReadTask(void *param);
void controlTask(void *param);

float get_alpha_ref(float alpha, float beta_actual, float depth_target);
double calculate_estimate_depth(double liftingange, double tailboardangle);

#endif // CONTROLLER_H
