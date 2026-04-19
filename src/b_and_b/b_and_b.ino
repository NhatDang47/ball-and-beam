#include "Config.h"
#include "PID.h"
#include "Motor.h"
#include "Sensors.h"
#include "SerialCommander.h"

// Setpoint mặc định
float SETPOINT_X = 17.0;    // cm

volatile bool newDataAvailable = false; 

// --- MODULES ---
Motor motor;
LaserSensor laser;
AngleSensor angleSensor;

// PID Configuration
PID pidPos(KP_X, KI_X, KD_X, -MAX_TILT, MAX_TILT, 0.05); //0.01
PID pidAngle(KP_TH, KI_TH, KD_TH, -255, 255, 0.01);

// --- State ---
enum SystemState { STATE_IDLE = 0, STATE_RUNNING = 1 };
volatile int currentState = STATE_IDLE;

// --- KHỞI TẠO COMMANDER ---
// Truyền địa chỉ của currentState vào để App Python có thể bật/tắt từ xa
SerialCommander commander(&SETPOINT_X, &currentState, &pidPos, 5.0, 45.0);

// Task
void TaskSensor(void *pvParameters);
void TaskControl(void *pvParameters);

void setup() {
    // Init serial
    commander.begin(115200);

    // Init IO
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);

    // Init Modules
    motor.init();
    laser.init();
    angleSensor.init();

    // Init Tasks
    // Core 1: sensors (Ưu tiên thấp hơn Control)
    xTaskCreatePinnedToCore(TaskSensor, "Sensor", 4096, NULL, 1, NULL, 1);
    
    // Core 0: PID & Motor (Ưu tiên cao nhất)
    xTaskCreatePinnedToCore(TaskControl, "Control", 4096, NULL, 5, NULL, 0);
}

void loop() {
    commander.update(); // Đọc UART

    // Feedback LED trạng thái
    if (currentState == STATE_RUNNING) {
        digitalWrite(PIN_LED_G, HIGH); digitalWrite(PIN_LED_R, LOW);
    } else {
        digitalWrite(PIN_LED_G, LOW); digitalWrite(PIN_LED_R, HIGH);
    }

    // Log UART gửi về App Python (Tần số 20Hz)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
        // App Python dùng đúng định dạng này để vẽ đồ thị
        Serial.printf("State:%d | Set:%.1f | Dist:%.2f | Ang:%.2f\n", 
                      currentState, SETPOINT_X, laser.getDistance(), angleSensor.getAngle());
        lastPrint = millis();
    }
}

// --- LOGIC ---
void stopSystem() {
    currentState = STATE_IDLE;
    motor.stop();
    digitalWrite(PIN_BUZZER, HIGH); delay(100); digitalWrite(PIN_BUZZER, LOW);
}

void startSystem() {
    angleSensor.reset(); 
    pidPos.reset();
    pidAngle.reset();
    currentState = STATE_RUNNING;

    // Start engine
    digitalWrite(PIN_BUZZER, HIGH); delay(50); digitalWrite(PIN_BUZZER, LOW);
    delay(50);
    digitalWrite(PIN_BUZZER, HIGH); delay(50); digitalWrite(PIN_BUZZER, LOW);
}

void handleButton() {
    static unsigned long pressStart = 0;
    static bool isPressed = false;
    int btnState = digitalRead(PIN_BUTTON);

    if (btnState == LOW && !isPressed) {
        isPressed = true;
        pressStart = millis();
    } else if (btnState == HIGH && isPressed) {
        unsigned long duration = millis() - pressStart;
        isPressed = false;

        if (duration > LONG_PRESS_MS) {
            if (currentState == STATE_IDLE) {
                motor.toggleDirection();
                digitalWrite(PIN_BUZZER, HIGH); delay(500); digitalWrite(PIN_BUZZER, LOW);
            }
        } else if (duration > 50) {
            if (currentState == STATE_IDLE) startSystem();
            else stopSystem();
        }
    }
}

// --- TASK CONTROL ---
void TaskControl(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Vòng lặp Góc 100Hz

    float targetAngle = 0.0; 

    for (;;) {
        handleButton();
        float currentDist = laser.getDistance(); 

        if (currentState == STATE_RUNNING) {
            float currentAngle = angleSensor.getAngle();
            
            if (abs(currentAngle) > MAX_TILT + 5.0) stopSystem();
            else {
                // ==========================================
                // VÒNG NGOÀI: TÍNH GÓC THAM CHIẾU (20HZ)
                // ==========================================
                if (newDataAvailable) {
                    newDataAvailable = false;
                    float ex = SETPOINT_X - currentDist;
                    float abs_ex = abs(ex);

                    // --- GAIN SCHEDULING ---
                    float dyn_Kp, dyn_Kd;
                    if (abs_ex >= 10.0f) {
                        dyn_Kp = KP_X;  
                        dyn_Kd = KD_X;
                    } else if (abs_ex > DEADBAND_X) {
                        float k_ratio = (abs_ex - DEADBAND_X) / (10.0f - DEADBAND_X);
                        dyn_Kp = 0.2f + k_ratio * (KP_X - 0.2f);
                        dyn_Kd = 0.5f + k_ratio * (KD_X - 0.5f);
                    } else {
                        dyn_Kp = 0.0f;
                        dyn_Kd = 0.0f;
                    }
                    
                    pidPos.setTunings(dyn_Kp, KI_X, dyn_Kd);
                    
                    // --- TÍNH TOÁN PD CASCADE ---
                    float a_req = pidPos.compute(SETPOINT_X, currentDist);
                    float ratio = a_req / K_SYS; 
                    ratio = constrain(ratio, -0.99f, 0.99f); 
                    float computedAngle = asin(ratio) * (180.0f / PI);
                    computedAngle = constrain(computedAngle, -MAX_TILT, MAX_TILT);

                    // --- BẪY FREEZE STATE ---
                    if (abs_ex <= DEADBAND_X) {
                        pidPos.reset(); 
                    } else {
                        targetAngle = computedAngle;
                    }
                }

                // ==========================================
                // VÒNG TRONG: ĐIỀU KHIỂN ĐỘNG CƠ (100Hz)
                // ==========================================
                if (abs(targetAngle - currentAngle) < 0.5) {
                    currentAngle = targetAngle;
                }
                
                float rawPWM = pidAngle.compute(targetAngle, currentAngle);

                int drivePWM = 0;
                if (rawPWM > 0) drivePWM = (int)rawPWM + PWM_MIN;
                else if (rawPWM < 0) drivePWM = (int)rawPWM - PWM_MIN;

                drivePWM = constrain(drivePWM, -MAX_PWM, MAX_PWM);
                motor.drive(drivePWM);
            }
        } 
        else { 
            motor.stop();
            targetAngle = 0;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- TASK SENSOR ---
void TaskSensor(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz

    for (;;) {
        laser.read(); 
        newDataAvailable = true; 
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}