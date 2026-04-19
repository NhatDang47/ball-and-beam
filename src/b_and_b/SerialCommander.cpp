#include "SerialCommander.h"

SerialCommander::SerialCommander(volatile float* setpointVar, volatile int* stateVar, PID* pidInstance, float minV, float maxV) {
    _targetSetpoint = setpointVar; _systemState = stateVar; _pidPos = pidInstance;
    _minVal = minV; _maxVal = maxV; _inputBuffer = "";
}

void SerialCommander::begin(unsigned long baudRate) { Serial.begin(baudRate); }

void SerialCommander::update() {
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            if (_inputBuffer.length() > 0) {
                _inputBuffer.trim();
                if (_inputBuffer.startsWith("S:")) {
                    float v = _inputBuffer.substring(2).toFloat();
                    if (v >= _minVal && v <= _maxVal) {
                        *_targetSetpoint = v;
                        if (_pidPos) _pidPos->reset();
                    }
                } else if (_inputBuffer.startsWith("P:")) {
                    *_systemState = _inputBuffer.substring(2).toInt();
                }
                _inputBuffer = "";
            }
        } else { _inputBuffer += inChar; }
    }
}