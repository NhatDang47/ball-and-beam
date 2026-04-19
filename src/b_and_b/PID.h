#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
private:
    float kp, ki, kd;
    float integral, prevError;
    float outMin, outMax;
    float dt; 
public:
    PID(float p, float i, float d, float minVal, float maxVal, float timeStep);
    float compute(float setpoint, float input);
    void reset();
    void setTunings(float p, float i, float d);
};
#endif