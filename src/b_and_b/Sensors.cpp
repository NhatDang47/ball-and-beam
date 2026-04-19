#include "Sensors.h"

// --- LASER ---
void LaserSensor::init() {
    Serial2.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
    node.begin(MODBUS_ID, Serial2);
    filteredDistance = SETPOINT_X;
}

void LaserSensor::read() {
    uint8_t result = node.readHoldingRegisters(MODBUS_REG, 1);
    if (result == node.ku8MBSuccess) {
        float raw = node.getResponseBuffer(0) / 10.0;
        // Lọc giá trị rác (chỉ nhận từ 1cm đến 50cm)
        if (raw > 1.0 && raw < 50.0) {
            filteredDistance = (1.0 - LPF_ALPHA) * filteredDistance + LPF_ALPHA * raw;
        }
    }
}

float LaserSensor::getDistance() {
    return filteredDistance;
}

// --- ENCODER ---
void AngleSensor::init() {
    encoder.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
    encoder.setCount(0);
}

float AngleSensor::getAngle() {
    float rawAngle = encoder.getCount() / PULSE_TO_DEG;
    // Trừ đi Offset để tìm điểm 0 thật
    return rawAngle - ANGLE_OFFSET;
}

void AngleSensor::reset() {
    encoder.setCount(0);
}