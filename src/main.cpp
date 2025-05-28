#include <Arduino.h>
#include <sbus.h>            // SBUS 接收器函式庫
#include <LS7366R.h>
#include "imu_mpu9255_utility.hpp"
#include "MotorController.hpp"

// Pin configurations
const std::array<uint8_t, 4> PIN_DIR = { 15, 14, 6, 28 };
const std::array<uint8_t, 4> PIN_PWM = { 25, 24, 9, 29 };
const std::array<int, 4> PWM_REVERSE = { -1, 1, -1, 1 };
const std::array<uint8_t, 4> PIN_QEI = { 30, 31, 33, 32 };

// PWM configuration parameters
const int CONFIG_ANALOG_WRITE_FREQ = 30000;  // PWM frequency in Hz
const int CONFIG_ANALOG_WRITE_RES = 12;      // PWM resolution in bits
const int MAX_PWM = (int)(pow(2, CONFIG_ANALOG_WRITE_RES)) - 1;  // Maximum PWM value based on resolution

// Motor control parameters
const double MOTOR_POWER_PERCENTAGE = 0.3;  // 30% power
const int FIXED_PWM = (int)(MAX_PWM * MOTOR_POWER_PERCENTAGE);  // Fixed PWM value

// Global objects
MPU9250 mpu9250(Wire, 0x68);
MotorController motorController(PIN_DIR, PIN_PWM, PWM_REVERSE, CONFIG_ANALOG_WRITE_RES);
LS7366R qei;
IEC::SBUS sbus(Serial1);

// IMU data structure
struct IMU {
    IMU() { memset(this, 0, sizeof(IMU)); }
    double accel[3];
    double gyro[3];
    double mag[3];
} imu;

// Encoder positions
long int pos[4];
long int last_pos[4] = {0, 0, 0, 0};

// Debug variables
unsigned long last_print_time = 0;
const unsigned long PRINT_INTERVAL = 100; // Print every 100ms

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize SBUS
    sbus.begin();
    
    // Initialize encoders
    qei.begin(PIN_QEI.data(), 4);
    
    // Initialize IMU
    imu_mpu9250_init(mpu9250);
    
    // Initialize motor controller
    motorController.init();
    
    // Reset all motors
    motorController.stopAll();
    
    Serial.println("System initialized!");
    Serial.print("Max PWM: "); Serial.println(MAX_PWM);
    Serial.print("Fixed PWM: "); Serial.println(FIXED_PWM);
    Serial.println("Direction configuration:");
    for (int i = 0; i < 4; i++) {
        Serial.print("Motor "); Serial.print(i);
        Serial.print(": "); Serial.println(PWM_REVERSE[i]);
    }
    delay(3000);
}

void loop() {
    // Read encoder positions
    qei.read();
    for (int i = 0; i < 4; i++) {
        last_pos[i] = pos[i];
        pos[i] = (i % 2 == 0) ? qei.get_pulse()[i] : -qei.get_pulse()[i];
    }

    // Update IMU data
    imu_mpu9250_update(mpu9250, imu.gyro, imu.accel, imu.mag);

    // Set each motor individually with proper direction
    for (int i = 0; i < 4; i++) {
        int power = FIXED_PWM * PWM_REVERSE[i];
        motorController.setPower(i, power);
    }

    // Debug printing
    unsigned long current_time = millis();
    if (current_time - last_print_time >= PRINT_INTERVAL) {
        last_print_time = current_time;
        
        // Print encoder positions and changes
        Serial.print("Encoder positions: ");
        for (int i = 0; i < 4; i++) {
            Serial.print(pos[i]);
            Serial.print(" (");
            Serial.print(pos[i] - last_pos[i]);
            Serial.print(") ");
        }
        
        // Print IMU data
        Serial.print(" | IMU Accel: ");
        for (int i = 0; i < 3; i++) {
            Serial.print(imu.accel[i]);
            Serial.print(" ");
        }
        
        // Print current power and direction
        Serial.print(" | Power: ");
        Serial.print(FIXED_PWM);
        Serial.print(" | Direction: ");
        for (int i = 0; i < 4; i++) {
            Serial.print(PWM_REVERSE[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
}

