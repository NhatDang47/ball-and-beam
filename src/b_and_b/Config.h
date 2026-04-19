#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- HARDWARE PINOUT ---
#define PIN_PWM1    22  
#define PIN_PWM2    23
#define PIN_ENABLE  21
#define PIN_ENC_A   18  
#define PIN_ENC_B   19
#define PIN_RX      16  
#define PIN_TX      17
#define PIN_BUTTON  32  
#define PIN_BUZZER  33  
#define PIN_LED_R   27  
#define PIN_LED_G   26  

// --- MODBUS CONFIG ---
#define MODBUS_ID   0x50
#define MODBUS_REG  0x34

const float DEADBAND_X = 0.2;

// --- PHYSICS ---
//  Thông số từ nhà sản xuất
const float GEAR_RATIO     = 2.0; 
const float ENCODER_PPR    = 1152.0;
const float PULSES_PER_REV = ENCODER_PPR * 4.0 * GEAR_RATIO;
const float PULSE_TO_DEG   = PULSES_PER_REV / 360.0;

// Len & Setpoint
const float BEAM_LENGTH = 50.0; 
extern float SETPOINT_X;

// --- SAFETY ---
const float SAFETY_DIST = 38.0; 
const float MAX_TILT    = 20.0; 

const int   MAX_PWM     = 255;
const int   PWM_MIN     = 21;    
const int   LONG_PRESS_MS = 500;

const float ANGLE_OFFSET = 0;

// --- PID ---
// 1. VÒNG NGOÀI (VỊ TRÍ - THIẾT KẾ BẰNG POLE PLACEMENT)
const float KP_X = 4.0;   // Tần số tự nhiên Wn^2
const float KI_X = 0.0;   // BẮT BUỘC BẰNG 0 (Hệ đã là Tích phân kép)
const float KD_X = 4.0;   // Suy giảm tới hạn 2*Zeta*Wn

// 2. VÒNG TRONG (GÓC - TINH CHỈNH THỰC NGHIỆM)
const float KP_TH = 2.4;  
const float KI_TH = 0.05;
const float KD_TH = 0.6;  

// --- HẰNG SỐ VẬT LÝ ---
const float K_SYS = 588.6;   // Gia tốc hệ thống 3g/5 (cm/s^2)

// Filter LPF
const float LPF_ALPHA = 0.65; 

#endif