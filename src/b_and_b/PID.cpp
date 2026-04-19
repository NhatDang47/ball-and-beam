#include "PID.h"

PID::PID(float p, float i, float d, float minVal, float maxVal, float timeStep) {
    kp = p; ki = i; kd = d;
    outMin = minVal; outMax = maxVal;
    dt = timeStep;
    integral = 0; prevError = 0;
}

float PID::compute(float setpoint, float input) {
    float error = setpoint - input;

    integral += error * dt;
    integral = constrain(integral, outMin, outMax);

    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = (kp * error) + (ki * integral) + (kd * derivative);
    return constrain(output, outMin, outMax);
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}

void PID::setTunings(float p, float i, float d) {
    kp = p; ki = i; kd = d;
}