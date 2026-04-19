#ifndef SERIAL_COMMANDER_H
#define SERIAL_COMMANDER_H
#include <Arduino.h>
#include "PID.h"

class SerialCommander {
private:
    float _minVal, _maxVal;
    String _inputBuffer; 
    volatile float* _targetSetpoint; 
    volatile int* _systemState; 
    PID* _pidPos;

public:
    SerialCommander(volatile float* setpointVar, volatile int* stateVar, PID* pidInstance, float minV, float maxV);
    void begin(unsigned long baudRate);
    void update();
};
#endif