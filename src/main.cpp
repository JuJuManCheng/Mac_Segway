#include <Arduino.h>
#include <sbus.h> // SBUS 接收器函式庫
#include <LS7366R.h>
#include "imu_mpu9255_utility.hpp"
#include "MotorController.hpp"
#include "Filter.hpp"

using namespace std;

// Hardware interface definitions
#define HW_SERIAL_INTERFACE_SBUS Serial1

#define voltage_pin A3
double voltage = 0.0;
// ===== Pin Configurations =====
const array<uint8_t, 4> PIN_DIR = {15, 14, 6, 28};    // Direction pins
const array<uint8_t, 4> PIN_PWM = {25, 24, 9, 29};    // PWM pins
const array<int, 4> PWM_REVERSE = {-1, 1, -1, 1};     // Motor direction multipliers
const array<uint8_t, 4> PIN_QEI = {30, 31, 33, 32};   // Quadrature encoder pins

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
const double T = 0.005;                                     // 100Hz control loop period
const double rc_fc = 50;
const double RC_FILTER_TAU = 1 / (2*PI*rc_fc);             // time constant for RC filtering
const double RC_FILTER_Alpha = exp(-T / RC_FILTER_TAU);    // Filter coefficient
const double vel_fc = 60;
const double VEL_FILTER_TAU = 1 / (2*PI*vel_fc);             // time constant for RC filtering
const double VEL_FILTER_Alpha = exp(-T / VEL_FILTER_TAU);    // Filter coefficient

// ===== Encoder Configuration =====
const double count_perRev = 1400.0;                        // Encoder counts per revolution

// ===== PID Configuration =====
const array<double, 4> Kp = {300.0, 300.0, 300.0, 300.0};  // Proportional gains
const array<double, 4> Ki = {1000.0, 1000.0, 1000.0, 1000.0};      // Integral gains
const array<double, 4> Kd = {0.0, 0.0, 0.0, 0.0};          // Derivative gains

// IMU Calibration parameters
// Replace these values with your calibration results from acc_calibration.npz
const double accel_offset[3] = {0.08584888, 0.06509522, 0.43634443}; 
const double accel_scale[3]  = {0.99995425, 1.00013127, 1.00013127}; 

const double gyro_offset[3] = {0.04281961, 0.18707194, 0.1028051}; 

const double mag_offset[3] = {0.08584888, 0.06509522, 0.43634443}; 
const double mag_scale[3]  = {0.99995425, 1.00013127, 1.00013127}; 

// ===== Global Objects =====
MPU9250 mpu9250(Wire, 0x68);
MotorController motorController(PIN_DIR, PIN_PWM, PWM_REVERSE, CONFIG_ANALOG_WRITE_RES);
LS7366R qei;
IEC::SBUS sbus(HW_SERIAL_INTERFACE_SBUS);

// ===== Filter Objects =====
// RC command filter
Filter::LowPassFilter1st rc_filter(RC_FILTER_Alpha);
Filter::LowPassFilter1st ang_vel_filter(VEL_FILTER_Alpha);

// ===== Global Variables =====
// RC Control
uint16_t rc[16] = {0};
double rc_to_velCommand = 0.0;
double pre_rc_to_velCommand = 0.0;
double filtered_rc_command = 0.0;
static int last_aux2 = -1; // 初始狀態為 -1 表示第一次執行

// Motor Control
std::array<double, 4> motor_power = {0.0, 0.0, 0.0, 0.0};

// IMU Data
struct IMU {
    IMU() { memset(this, 0, sizeof(IMU)); }
    double accel[3];
    double gyro[3];
    double mag[3];
    double filtered_accel[3];
    double filtered_gyro[3];
} imu;

// Encoder Data
array<double, 4> pos = {0.0, 0.0, 0.0, 0.0};
array<double, 4> last_pos = {0.0, 0.0, 0.0, 0.0};
array<double, 4> ang_vel = {0.0, 0.0, 0.0, 0.0};
array<double, 4> ang_vel_prev = {0.0, 0.0, 0.0, 0.0};
array<double, 4> filtered_ang_vel = {0.0, 0.0, 0.0, 0.0};

// PID Variables
array<double, 4> e = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_integral = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_derivative = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_prev = {0.0, 0.0, 0.0, 0.0};

// Timer and Debug
IntervalTimer timer;
volatile unsigned long interruptCount = 0.0;
unsigned long last_print_time = 0.0;
const unsigned long PRINT_INTERVAL = 100.0;

// RC channel mapping
enum RC_CHANNEL {
    CH_ROLL = 0,        // 左右
    CH_THROTTLE = 1,    // 油門
    CH_PITCH = 2,       // 前後
    CH_YAW = 3,         // 轉向
    CH_MODE = 4,        // 模式選擇
    CH_AUX1 = 5,        // 輔助開關1
    CH_AUX2 = 6,        // 輔助開關2
    CH_AUX3 = 7         // 輔助開關3
};

// RC command structure
struct RC_Command {
    double throttle;    // 油門值
    double roll;        // 左右值
    double pitch;       // 前後值
    double yaw;         // 轉向值
    int mode;          // 模式
    bool aux1;         // 輔助開關1狀態
    int aux2;         // 輔助開關2狀態
    bool aux3;         // 輔助開關3狀態
};

RC_Command rc_command;

// ===== Helper Functions =====
double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

// ===== Control Functions =====
void reset() {
    rc_to_velCommand = 0.0;
    pre_rc_to_velCommand = 0.0;

    e.fill(0.0);
    e_integral.fill(0.0);
    e_derivative.fill(0.0);
    e_prev.fill(0.0);
}

double low_pass_filter(double new_value, double pre_val){
    double filtered_val = VEL_FILTER_Alpha * new_value + (1 - VEL_FILTER_Alpha) * pre_val;
    return filtered_val;
}

void get_vel(array<double, 4>& pos, array<double, 4>& last_pos, array<double, 4>& ang_vel) {
    for (int j = 0; j < 4; j++) {
        ang_vel[j] = (pos[j] - last_pos[j]) * 2.0 * PI / (T * count_perRev);
        filtered_ang_vel[j] = low_pass_filter(ang_vel[j], filtered_ang_vel[j]);
        ang_vel_prev[j] = ang_vel[j];
        last_pos[j] = pos[j];
    }
}

double RC_Process_command(uint16_t* rc_values) {
    // 處理油門通道 (CH_THROTTLE)
    if (rc_values[CH_THROTTLE] <= RC_CENTER) {
        rc_command.throttle = map(rc_values[CH_THROTTLE], RC_MIN, RC_CENTER, MAX_SPEED, 0);
    } else {
        rc_command.throttle = map(rc_values[CH_THROTTLE], RC_CENTER, RC_MAX, 0, -MAX_SPEED);
    }
    
    if (rc_values[CH_AUX2] < 800) {
        rc_command.aux2 = 1;
    }
    else if(rc_values[CH_AUX2] > 1200){
        rc_command.aux2 = 2;
    }
    else{
        rc_command.aux2 = 0;
    }

    if(last_aux2 != rc_command.aux2){
        reset();
        last_aux2 = rc_command.aux2;
    }

    // 處理左右通道 (CH_ROLL)
    rc_command.roll = map(rc_values[CH_ROLL], 200, 1796, -MAX_PWM, MAX_PWM);
    
    // 處理模式選擇 (CH_MODE)
    if (rc_values[CH_MODE] < RC_CENTER - 200) {
        rc_command.mode = 1;  // 模式1
    } else if (rc_values[CH_MODE] > RC_CENTER + 200) {
        rc_command.mode = 2;  // 模式2
    } else {
        rc_command.mode = 0;  // 模式0
    }
    
    if(rc_command.aux2 == 2){
        // 根據模式選擇不同的控制策略
        switch (rc_command.mode) {
            case 0:  // 正常模式
                rc_to_velCommand = rc_command.throttle;
                break;
                
            case 1:  // 精確模式
                rc_to_velCommand = rc_command.throttle * 0.5;  // 降低速度
                break;
                
            case 2:  // 運動模式
                rc_to_velCommand = rc_command.throttle * 1.5;  // 提高速度
                break;
        }
        
        // 應用低通濾波
        // rc_to_velCommand = rc_filter.update(rc_to_velCommand);
        filtered_rc_command = low_pass_filter(rc_to_velCommand, filtered_rc_command);
        
        // 死區處理
        if (abs(filtered_rc_command) < 1.0) {
            filtered_rc_command = 0.0;
        }
    }
    else if(rc_command.aux2 == 1){
        filtered_rc_command = rc_command.roll;
    }
    else{
        filtered_rc_command = 0.0;
    }
    
    return filtered_rc_command;
}

void PID_controller(double rc_to_velCommand) {
    for (int j = 0; j < 4; j++) {
        // Calculate error terms
        e[j] = rc_to_velCommand - filtered_ang_vel[j];
        
        // Reset integral when target speed sign changes
        if (sign(rc_to_velCommand) != sign(pre_rc_to_velCommand)) {
            e_integral[j] = 0;
        }
        
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
        // motor_power[j] = KP + KI + KD;
        
        // Update previous error
        e_prev[j] = e[j];
        if (sign(rc_to_velCommand) != sign(pre_rc_to_velCommand) &&
            abs(rc_to_velCommand) > 1.0) {
            e_integral[j] = 0;
        }
    }
}

void print_data(){
    Serial.print(rc_to_velCommand, 3);
    Serial.print(" ");

    for(int i = 0; i < 4; i++){
        Serial.print(ang_vel[i], 3);
        Serial.print(" ");
        Serial.print(filtered_ang_vel[i], 3);
        Serial.print(" ");
    }
    Serial.println();
}

// ===== Timer Callback =====
void timerCallback() {
    
    // Process RC command
    rc_to_velCommand = RC_Process_command(rc);

    // Calculate angular velocities
    get_vel(pos, last_pos, ang_vel);
    if (rc_command.aux2 == 2) {  // 使用 aux2 作為啟動開關

        // Apply PID control
        PID_controller(rc_to_velCommand);
        pre_rc_to_velCommand = rc_to_velCommand;
    } 
    else if (rc_command.aux2 == 1){
        motor_power = {0.0, 0.0, rc_to_velCommand, 0.0};
    }
    else {
        // Stop motors if RC switch is off
        motor_power = {0.0, 0.0, 0.0, 0.0};
        reset();
    }
    
    print_data();
}

// ===== Setup and Loop =====
void setup() {
    Serial.begin(115200);               // Initialize serial communication
    sbus.begin();                       // Initialize SBUS
    qei.begin(PIN_QEI.data(), 4);       // Initialize encoders 
    imu_mpu9250_init(mpu9250);          // Initialize IMU
    pinMode(voltage_pin, INPUT);        // Initialize voltage pin
    analogReadResolution(12);           // Set analog read resolution to 12 bits
    motorController.init();             // Initialize motor controller
    motorController.stopAll();
    reset();
    
    timer.begin(timerCallback, T*1000000);  // 1000000 microseconds = 100Hz   
    Serial.println("System initialized!");
    Serial.println("Direction configuration:");
    delay(3000);
}

void loop() {
    // Read SBUS data
    sbus.read(rc);
    voltage = 0.0078 * (double)(analogRead(voltage_pin)); // Convert analog reading to voltage
    // Read encoder positions
    qei.read();
    for (int i = 0; i < 4; i++) {
        pos[i] = (i % 2 == 0) ? qei.get_pulse()[i] : -qei.get_pulse()[i];
    }
    
    // Update IMU data
    imu_mpu9250_update(mpu9250, imu.gyro, imu.accel, imu.mag);

    // === Apply accelerometer calibration ===

    for (int i = 0; i < 3; ++i) {
        imu.accel[i] = accel_scale[i] * (imu.accel[i] - accel_offset[i]);
        imu.gyro[i] = imu.gyro[i] - gyro_offset[i];
        imu.mag[i] = mag_scale[i] * (imu.mag[i] - mag_offset[i]);
    }

    // Update motor powers
    for (int i = 0; i < 4; i++) {
        motorController.setPower(i, motor_power[i]);
    }
}
