#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Config.h"

class Motor {
private:
    int directionSign = -1; // 1: Thuận, -1: Nghịch
public:
    void init();
    void drive(int controlSignal);
    void stop();
    void toggleDirection();
    int getDirection();
};
#endif