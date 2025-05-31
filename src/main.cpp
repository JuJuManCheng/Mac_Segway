#include <Arduino.h>
#include <sbus.h> // SBUS 接收器函式庫
#include <LS7366R.h>
#include "imu_mpu9255_utility.hpp"
#include "MotorController.hpp"

// Hardware interface definitions
#define HW_SERIAL_INTERFACE_SBUS Serial1

// ===== Pin Configurations =====
const std::array<uint8_t, 4> PIN_DIR = {15, 14, 6, 28};    // Direction pins
const std::array<uint8_t, 4> PIN_PWM = {25, 24, 9, 29};    // PWM pins
const std::array<int, 4> PWM_REVERSE = {-1, 1, -1, 1};     // Motor direction multipliers
const std::array<uint8_t, 4> PIN_QEI = {30, 31, 33, 32};   // Quadrature encoder pins

// ===== PWM Configuration =====
const int CONFIG_ANALOG_WRITE_FREQ = 30000;                // PWM frequency in Hz
const int CONFIG_ANALOG_WRITE_RES = 12;                    // PWM resolution in bits
const int MAX_PWM = (int)(pow(2, CONFIG_ANALOG_WRITE_RES)) - 1;  // Maximum PWM value

// ===== RC Input Configuration =====
const uint16_t RC_MIN = 200;                               // Minimum RC input value
const uint16_t RC_MAX = 1800;                              // Maximum RC input value
const uint16_t RC_CENTER = 1046;                           // Center RC input value
const double MAX_SPEED = 30.0;                             // Maximum speed in rad/s

// ===== Control Loop Configuration =====
const double T = 0.01;                                     // 100Hz control loop period
const double RC_FILTER_TAU = 0.1;                          // 100ms time constant for RC filtering
const double RC_FILTER_Alpha = exp(-T / RC_FILTER_TAU);    // Filter coefficient

// ===== Encoder Configuration =====
const double count_perRev = 1400.0;                        // Encoder counts per revolution

// ===== PID Configuration =====
const std::array<double, 4> Kp = {200.0, 200.0, 200.0, 200.0};  // Proportional gains
const std::array<double, 4> Ki = {50.0, 50.0, 50.0, 50.0};      // Integral gains
const std::array<double, 4> Kd = {0.0, 0.0, 0.0, 0.0};          // Derivative gains

// ===== Global Objects =====
MPU9250 mpu9250(Wire, 0x68);
MotorController motorController(PIN_DIR, PIN_PWM, PWM_REVERSE, CONFIG_ANALOG_WRITE_RES);
LS7366R qei;
IEC::SBUS sbus(HW_SERIAL_INTERFACE_SBUS);

// ===== Global Variables =====
// RC Control
uint16_t rc[16] = {0};
volatile uint16_t rc1_forInterrupt = RC_CENTER;
volatile uint16_t rc7_forInterrupt = 1000;
double filtered_rc_command = 0.0;
double rc_to_velCommand = 0.0;

// Motor Control
std::array<double, 4> motor_power = {0.0, 0.0, 0.0, 0.0};

// IMU Data
struct IMU {
    IMU() { memset(this, 0, sizeof(IMU)); }
    double accel[3];
    double gyro[3];
    double mag[3];
} imu;

// Encoder Data
std::array<double, 4> pos = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> last_pos = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> ang_vel = {0.0, 0.0, 0.0, 0.0};

// PID Variables
std::array<double, 4> e = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> e_integral = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> e_derivative = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> e_prev = {0.0, 0.0, 0.0, 0.0};

// Timer and Debug
IntervalTimer timer100Hz;
volatile unsigned long interruptCount = 0.0;
double i = 0.0;
double t = 0.0;
unsigned long last_print_time = 0.0;
const unsigned long PRINT_INTERVAL = 100.0;

// ===== Helper Functions =====
double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

// ===== Control Functions =====
void reset() {
    interruptCount = 0.0;
    i = 0.0;
    t = 0.0;
    
    e.fill(0.0);
    e_integral.fill(0.0);
    e_derivative.fill(0.0);
    e_prev.fill(0.0);
    
    pos.fill(0.0);
    last_pos.fill(0.0);
    ang_vel.fill(0.0);
}

void get_vel(const std::array<double, 4>& current_pos,
            const std::array<double, 4>& prev_pos,
            std::array<double, 4>& velocity) {
    for (int j = 0; j < 4; j++) {
        velocity[j] = (current_pos[j] - prev_pos[j]) * 2.0 * PI / (T * count_perRev);
    }
}

double low_pass_filter(double new_command) {
    filtered_rc_command = RC_FILTER_Alpha * filtered_rc_command + 
                         (1 - RC_FILTER_Alpha) * new_command;
    return filtered_rc_command;
}

double RC_Process_command(uint16_t rc_value) {
    // Map RC input to velocity command
    if (rc_value <= RC_CENTER) {
        rc_to_velCommand = map(rc_value, RC_MIN, RC_CENTER, MAX_SPEED, 0);
    } else {
        rc_to_velCommand = map(rc_value, RC_CENTER, RC_MAX, 0, -MAX_SPEED);
    }

    // Apply low-pass filter and deadband
    rc_to_velCommand = low_pass_filter(rc_to_velCommand);
    if (abs(rc_to_velCommand) < 5.0) {
        rc_to_velCommand = 0.0;
    }

    return rc_to_velCommand;
}

void PID_controller(double rc_to_velCommand) {
    for (int j = 0; j < 4; j++) {
        // Calculate error terms
        e[j] = rc_to_velCommand - ang_vel[j];
        e_integral[j] += e[j] * T;
        e_derivative[j] = (e[j] - e_prev[j]) / T;
        
        // Calculate PID terms
        double KP = Kp[j] * e[j];
        double KI = Ki[j] * e_integral[j];
        double KD = Kd[j] * e_derivative[j];
        
        // Anti-windup for integral term
        if (abs(KI) >= 0.6 * MAX_PWM) {
            KI = 0.6 * MAX_PWM * sign(KI);
        }
        
        // Calculate motor power with direction
        motor_power[j] = (KP + KI + KD) * PWM_REVERSE[j];
        
        // Update previous error
        e_prev[j] = e[j];
    }
}

// ===== Timer Callback =====
void timerCallback() {
    i++;
    t = i * T;
    
    // Process RC command
    rc_to_velCommand = RC_Process_command(rc1_forInterrupt);
    
    if (rc7_forInterrupt > 1000) {
        // Calculate angular velocities
        get_vel(pos, last_pos, ang_vel);
        
        // Apply PID control
        PID_controller(rc_to_velCommand);
    } else {
        // Stop motors if RC switch is off
        motor_power.fill(0.0);
        reset();
    }
    
    interruptCount++;
}

// ===== Setup and Loop =====
void setup() {

    Serial.begin(115200);               // Initialize serial communication
    sbus.begin();                       // Initialize SBUS
    qei.begin(PIN_QEI.data(), 4);       // Initialize encoders 
    imu_mpu9250_init(mpu9250);          // Initialize IMU
    motorController.init();             // Initialize motor controller
    motorController.stopAll();
    
    timer100Hz.begin(timerCallback, 10000);  // 10000 microseconds = 100Hz    
    Serial.println("System initialized!");
    Serial.println("Direction configuration:");
    delay(3000);
}

void loop() {
    // Read SBUS data
    sbus.read(rc);
    noInterrupts();
    rc1_forInterrupt = rc[1];
    rc7_forInterrupt = rc[6];
    interrupts();
    
    // Read encoder positions
    qei.read();
    for (int i = 0; i < 4; i++) {
        pos[i] = (i % 2 == 0) ? qei.get_pulse()[i] : -qei.get_pulse()[i];
    }
    
    // Update IMU data
    imu_mpu9250_update(mpu9250, imu.gyro, imu.accel, imu.mag);
    
    // Update motor powers
    for (int i = 0; i < 4; i++) {
        motorController.setPower(i, motor_power[i]);
    }
}
