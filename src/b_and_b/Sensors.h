#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ModbusMaster.h>
#include "Config.h"

class LaserSensor {
private:
    ModbusMaster node;
    float filteredDistance;
public:
    void init();
    void read();
    float getDistance();
};

class AngleSensor {
private:
    ESP32Encoder encoder;
public:
    void init();
    float getAngle();
    void reset();
};
#endif