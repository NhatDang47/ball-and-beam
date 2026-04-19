#include "Motor.h"

void Motor::init() {
    pinMode(PIN_ENABLE, OUTPUT);
    digitalWrite(PIN_ENABLE, HIGH);
    ledcAttach(PIN_PWM1, 15000, 8); // 15kHz, 8-bit
    ledcAttach(PIN_PWM2, 15000, 8);
}

void Motor::drive(int controlSignal) {
    int effectiveSignal = controlSignal * directionSign;
    
    int pwmOut = abs(effectiveSignal);
    if (pwmOut > 0) pwmOut += PWM_MIN;
    pwmOut = constrain(pwmOut, 0, MAX_PWM);

    if (effectiveSignal > 0) {
        ledcWrite(PIN_PWM1, pwmOut);
        ledcWrite(PIN_PWM2, 0);
    } else if (effectiveSignal < 0) {
        ledcWrite(PIN_PWM1, 0);
        ledcWrite(PIN_PWM2, pwmOut);
    } else {
        stop();
    }
}

void Motor::stop() {
    ledcWrite(PIN_PWM1, 0);
    ledcWrite(PIN_PWM2, 0);
}

void Motor::toggleDirection() {
    directionSign = -directionSign;
}

int Motor::getDirection() {
    return directionSign;
}