#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <array>
#include <cmath>
#include "imu_interface.hpp"
#include <sbus.h> // SBUS 接收器函式庫
#include <LS7366R.h>
#include "imu_mpu9255_utility.hpp"
#include "MotorController.hpp"
#include <iostream>
#include <vector>
#include "../lib/orientation/orientation.hpp"
#include <simple_timer.hpp>
#include "cmd_prefilter.hpp"

using namespace std;

// Hardware interface definitions
#define HW_SERIAL_INTERFACE_SBUS Serial1

#define voltage_pin A3
double voltage = 0.0;
// ===== Pin Configurations =====
const array<uint8_t, 4> PIN_DIR = {15, 14, 6, 28};    // Direction pins
const array<uint8_t, 4> PIN_PWM = {25, 24, 9, 29};    // PWM pins
const array<int, 4> PWM_REVERSE = {-1, 1, -1, 1};
const array<std::array<double, 3>, 4> PWM_MAP_MATRIX = {{
    array<double, 3>{1.0, 1.0, 1.0},
    array<double, 3>{-1.0, 1.0, 1.0},
    array<double, 3>{1.0,  1.0, -1.0},
    array<double, 3>{-1.0, 1.0, -1.0}
}};

const array<uint8_t, 4> PIN_QEI = {30, 31, 33, 32};   // Quadrature encoder pins

// ===== PWM Configuration =====
const int CONFIG_ANALOG_WRITE_FREQ = 30000;                // PWM frequency in Hz
const int CONFIG_ANALOG_WRITE_RES = 12;                    // PWM resolution in bits
const int MAX_PWM = (int)(pow(2, CONFIG_ANALOG_WRITE_RES)) - 1;  // Maximum PWM value

// ===== Madgwick AHRS & Orientation Integration =====
#define AHRS_UPDATE_PERIOD_MICROS 1250 // 姿態解算更新週期（微秒）
double T_AHRS  = (double)(AHRS_UPDATE_PERIOD_MICROS) / 1000000.0;
Orientation orient; // 姿態解算物件
imu_data_t imu_raw; // 原始IMU資料結構
vector<float> orientation_buffer; // 暫存姿態資料用

// ===== SimpleTimer 物件宣告 =====
SimpleTimer ahrs_timer, serial_timer;


// ===== RC Input Configuration =====
const uint16_t RC_MIN = 200;                               // Minimum RC input value
const uint16_t RC_MAX = 1800;                              // Maximum RC input value
const uint16_t RC_CENTER = 1000;                           // Center RC input value
const double MAX_MOTOR_SPEED = 30.0;                             // Maximum speed in rad/s
const double MAX_PITCH = 10.0;                             // Maximum pitch in rad/s
const double MAX_PITCH_RATE = 3.0; // rad/s

// ===== Control Loop Configuration =====
const double T = 0.005;                                      
const double rc_fc = 50;
const double RC_FILTER_TAU = 1 / (2*PI*rc_fc);             // time constant for RC filtering
const double RC_FILTER_Alpha = exp(-T / RC_FILTER_TAU);    // Filter coefficient
const double vel_fc = 60;
const double VEL_FILTER_TAU = 1 / (2*PI*vel_fc);             // time constant for RC filtering
const double VEL_FILTER_Alpha = exp(-T / VEL_FILTER_TAU);    // Filter coefficient

// ===== Encoder Configuration =====
const double count_perRev = 1400.0;                        // Encoder counts per revolution

// ===== MotorPID Configuration =====
const array<double, 4> MOTOR_Kp = {300.0, 300.0, 300.0, 300.0};  // Proportional gains
const array<double, 4> MOTOR_Ki = {1000.0, 1000.0, 1000.0, 1000.0};      // Integral gains
const array<double, 4> MOTOR_Kd = {0.0, 0.0, 0.0, 0.0};          // Derivative gains

// RC for Segway Control
double SEG_PITCH_COMMAND = 0.0;
double PRE_SEG_PITCH_COMMAND = 0.0;
double FILTERED_SEG_PITCH_COMMAND = 0.0;

double SEG_PITCH_RATE_COMMAND = 0.0;
double PRE_SEG_PITCH_RATE_COMMAND = 0.0;
double FILTERED_SEG_PITCH_RATE_COMMAND = 0.0;

// ===== Attitude =====
array<double, 3> attitude = {0.0, 0.0, 0.0}; // attitude[0]=roll, [1]=pitch, [2]=yaw
array<double, 3> Segway_ang_vel = {0.0, 0.0, 0.0}; // Segway_ang_vel[0]=roll rate, [1]=pitch rate, [2]=yaw rate
array<double, 3> Segway_accel = {0.0, 0.0, 0.0}; // Segway_accel[0]=x acc, [1]=y acc, [2]=z acc

// ===== Segway PID Configuration =====
const double SEG_PITCH_RATE_Kp = -4.5;
const double SEG_PITCH_RATE_Ki = -1.0;      // Integral gains
const double SEG_PITCH_RATE_Kd = 0.0;
double SEG_PITCH_RATE_ERROR = 0.0;
double SEG_PITCH_RATE_ERROR_INTEGRAL = 0.0;
double SEG_PITCH_RATE_ERROR_DERIVATIVE = 0.0;
double SEG_PITCH_RATE_ERROR_PREV = 0.0;

const double SEG_PITCH_Kp = 40.0;
const double SEG_PITCH_Ki = 2.0;      // Integral gains
const double SEG_PITCH_Kd = 0.0;          // Derivative gains
double SEG_PITCH_ERROR = 0.0;
double SEG_PITCH_ERROR_INTEGRAL = 0.0;
double SEG_PITCH_ERROR_DERIVATIVE = 0.0;
double SEG_PITCH_ERROR_PREV = 0.0;

double SEG_PITCH_TO_PITCH_RATE_CMD = 0.0;
double FILTERED_SEG_PITCH_TO_PITCH_RATE_CMD = 0.0;
double PRE_SEG_PITCH_TO_PITCH_RATE_CMD = 0.0;

array<double, 3> SEG_CONTROL_LAW = {0.0, 0.0, 0.0};

// ===== Global Objects =====
MPU9250 mpu9250(Wire, 0x68);
MotorController motorController(PIN_DIR, PIN_PWM, PWM_REVERSE, CONFIG_ANALOG_WRITE_RES);
LS7366R qei;
IEC::SBUS sbus(HW_SERIAL_INTERFACE_SBUS);
cmd_prefilter CMD1;

// ===== Global Variables =====
// RC for motor Control
uint16_t rc[16] = {0};
array<double, 4> wheel_command = {0.0, 0.0, 0.0, 0.0};
array<double, 4> pre_wheel_command = {0.0, 0.0, 0.0, 0.0};
array<double, 4> filtered_wheel_command = {0.0, 0.0, 0.0, 0.0};
static int last_aux2 = -1; // 初始狀態為 -1 表示第一次執行

// Motor Control
std::array<double, 4> motor_power = {0.0, 0.0, 0.0, 0.0};

// Encoder Data
array<double, 4> wheel_pos = {0.0, 0.0, 0.0, 0.0};
array<double, 4> wheel_last_pos = {0.0, 0.0, 0.0, 0.0};
array<double, 4> wheel_ang_vel = {0.0, 0.0, 0.0, 0.0};
array<double, 4> wheel_ang_vel_prev = {0.0, 0.0, 0.0, 0.0};
array<double, 4> filtered_wheel_ang_vel = {0.0, 0.0, 0.0, 0.0};

// Motor PID Variables
array<double, 4> e = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_integral = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_derivative = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_prev = {0.0, 0.0, 0.0, 0.0};

// Timer and Debug
IntervalTimer timer;

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
    double lateral;        // 左右值
    double pitch;       // 前後值
    double yaw;         // 轉向值
    int mode;          // 模式
    bool aux1;         // 輔助開關1狀態
    int aux2;         // 輔助開關2狀態
    bool aux3;         // 輔助開關3狀態
};

RC_Command rc_command;

// ===== Helper Functions =====
double sign(double x);
double mapf(int x, double in_min, double in_max, double out_min, double out_max);
void reset();
double low_pass_filter(double new_value, double pre_val);
void get_vel(array<double, 4>& wheel_pos, array<double, 4>& wheel_last_pos, array<double, 4>& wheel_ang_vel);
void RC_Process_command(uint16_t* rc_values);
void motor_controller(array<double, 4>& filtered_wheel_command);
void Segway_Pitch_Rate_controller(double FILTERED_SEG_PITCH_RATE_COMMAND);
void Segway_Pitch_controller(double FILTERED_SEG_PITCH_COMMAND);
void print_data();
void routine();
void attitude_init();
void get_sensor_data(array<double, 4>& wheel_pos, array<double, 4>& wheel_last_pos, array<double, 4>& wheel_ang_vel, array<double, 3>& Segway_ang_vel, array<double, 3>& Segway_accel, Orientation& orient);

int elapTime = 0;
// ===== Setup and Loop =====
void setup() {
    Serial.begin(115200);               // 初始化序列埠
    sbus.begin();                       // 初始化SBUS
    qei.begin(PIN_QEI.data(), 4);       // 初始化編碼器
    imu_mpu9250_init(mpu9250);          // 初始化IMU
    pinMode(voltage_pin, INPUT);        // 初始化電壓腳位
    analogReadResolution(12);           // 設定類比讀取解析度為12位元
    motorController.init();             // 初始化馬達控制器
    motorController.stopAll();          // 停止所有馬達
    timer.begin(routine, T*1000000);    
    // ===== 姿態解算初始化 =====
    attitude_init();
    ahrs_timer.begin();
    reset();                            // 重設控制變數
    Serial.println("System initialized!");
    Serial.println("Direction configuration:");
    delay(1000);
}

void loop() {
    // 讀取SBUS資料
    sbus.read(rc);
    voltage = 0.0078 * (double)(analogRead(voltage_pin)); // 類比電壓轉換
    // 讀取編碼器位置
    qei.read();
    for (int i = 0; i < 4; i++) {
        wheel_pos[i] = (i % 2 == 0) ? qei.get_pulse()[i] : -qei.get_pulse()[i];
    }
    // ===== 姿態解算與資料傳送 =====
    // 更新IMU資料（給姿態解算用）
    imu_mpu9250_update(mpu9250, imu_raw.gyro, imu_raw.accel, imu_raw.mag);
    // 依指定週期更新姿態解算
    if (ahrs_timer.elapsed() >= AHRS_UPDATE_PERIOD_MICROS) {
        ahrs_timer.begin();
        orient.update(imu_raw); // 姿態解算更新
        elapTime = ahrs_timer.get_time_duration();
    }
    
    // 更新馬達輸出
    for (int i = 0; i < 4; i++) {
        motorController.setPower(i, motor_power[i]);
    }
}

void routine() {
    // Process RC command
    RC_Process_command(rc);

    get_sensor_data(wheel_pos, wheel_last_pos, wheel_ang_vel, Segway_ang_vel, Segway_accel, orient);

    if (rc_command.aux2 == 2) {  // 使用 aux2 作為啟動開關
        switch (rc_command.mode) {
            case 0:  // PITCH RATE CONTROL
                Segway_Pitch_Rate_controller(FILTERED_SEG_PITCH_RATE_COMMAND);
                PRE_SEG_PITCH_RATE_COMMAND = FILTERED_SEG_PITCH_RATE_COMMAND;
                motor_controller(filtered_wheel_command);
                pre_wheel_command = filtered_wheel_command;
                break;
            case 1:  // ATTITUDE CONTROL (PITCH)
                Segway_Pitch_controller(FILTERED_SEG_PITCH_COMMAND);
                PRE_SEG_PITCH_COMMAND = FILTERED_SEG_PITCH_COMMAND;
                Segway_Pitch_Rate_controller(FILTERED_SEG_PITCH_TO_PITCH_RATE_CMD);
                PRE_SEG_PITCH_RATE_COMMAND = FILTERED_SEG_PITCH_TO_PITCH_RATE_CMD;
                motor_controller(filtered_wheel_command);
                pre_wheel_command = filtered_wheel_command;
                break;
            case 2:  // position control
                
                break;
        }
    } 
    else if (rc_command.aux2 == 1){
        motor_power = {0.0, 0.0, 0.0, 0.0};
        reset();
    }
    else {
        switch (rc_command.mode) {
            case 0:  
                motor_controller(filtered_wheel_command);
                pre_wheel_command = filtered_wheel_command;
                break;
            case 1:  
                motor_controller(filtered_wheel_command);
                pre_wheel_command = filtered_wheel_command;
                break;
            case 2: 
                motor_controller(filtered_wheel_command);
                pre_wheel_command = filtered_wheel_command;
                break;
        }
    }
    
    print_data();
}

// ===== Function Definitions (after setup/loop) =====
double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

double mapf(int x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void reset() {
    filtered_wheel_command.fill(0.0);
    pre_wheel_command.fill(0.0);
    wheel_command.fill(0.0);

    SEG_PITCH_RATE_COMMAND = 0.0;
    PRE_SEG_PITCH_RATE_COMMAND = 0.0;
    FILTERED_SEG_PITCH_RATE_COMMAND = 0.0;

    SEG_PITCH_RATE_ERROR = 0.0;
    SEG_PITCH_RATE_ERROR_INTEGRAL = 0.0;
    SEG_PITCH_RATE_ERROR_DERIVATIVE = 0.0;
    SEG_PITCH_RATE_ERROR_PREV = 0.0;

    SEG_PITCH_COMMAND = 0.0;
    PRE_SEG_PITCH_COMMAND = 0.0;
    FILTERED_SEG_PITCH_COMMAND = 0.0;

    SEG_PITCH_TO_PITCH_RATE_CMD = 0.0;
    FILTERED_SEG_PITCH_TO_PITCH_RATE_CMD = 0.0;

    SEG_PITCH_ERROR = 0.0;
    SEG_PITCH_ERROR_INTEGRAL = 0.0;
    SEG_PITCH_ERROR_DERIVATIVE = 0.0;
    SEG_PITCH_ERROR_PREV = 0.0;

    SEG_CONTROL_LAW.fill(0.0);

    e.fill(0.0);
    e_integral.fill(0.0);
    e_derivative.fill(0.0);
    e_prev.fill(0.0);
}

double low_pass_filter(double new_value, double pre_val){
    double filtered_val = VEL_FILTER_Alpha * new_value + (1 - VEL_FILTER_Alpha) * pre_val;
    return filtered_val;
}

void get_sensor_data(array<double, 4>& wheel_pos, array<double, 4>& wheel_last_pos, array<double, 4>& wheel_ang_vel, array<double, 3>& Segway_ang_vel, array<double, 3>& Segway_accel, Orientation& orient){
    get_vel(wheel_pos, wheel_last_pos, wheel_ang_vel); // Wheel angular velocity

    // Segway attitude
    const double* euler = orient.get_euler();
    attitude[0] = euler[0] * RAD_TO_DEG; // roll
    attitude[1] = euler[1] * RAD_TO_DEG; // pitch
    attitude[2] = euler[2] * RAD_TO_DEG; // yaw
    
    //IMU data
    Segway_accel[0] = orient.get_filtered_accel()[0]; // x acc
    Segway_accel[1] = orient.get_filtered_accel()[1]; // y acc
    Segway_accel[2] = orient.get_filtered_accel()[2]; // z acc
    Segway_ang_vel[0] = orient.get_filtered_gyro()[0]; // roll rate
    Segway_ang_vel[1] = orient.get_filtered_gyro()[1]; // pitch rate
    Segway_ang_vel[2] = orient.get_filtered_gyro()[2]; // yaw rate

}

void get_vel(array<double, 4>& wheel_pos, array<double, 4>& wheel_last_pos, array<double, 4>& wheel_ang_vel) {
    for (int j = 0; j < 4; j++) {
        wheel_ang_vel[j] = (wheel_pos[j] - wheel_last_pos[j]) * 2.0 * PI / (T * count_perRev);
        filtered_wheel_ang_vel[j] = low_pass_filter(wheel_ang_vel[j], filtered_wheel_ang_vel[j]);
        wheel_ang_vel_prev[j] = wheel_ang_vel[j];
        wheel_last_pos[j] = wheel_pos[j];
    }
}

void RC_Process_command(uint16_t* rc_values) {
    // aux2: 0=normal, 1=reset, 2=Segway control
    if (rc_values[CH_AUX2] < 800) {
        rc_command.aux2 = 0;
    } else if (rc_values[CH_AUX2] > 1200) {
        rc_command.aux2 = 2;
    } else {
        rc_command.aux2 = 1;
    }

    if(last_aux2 != rc_command.aux2){
        reset();
        last_aux2 = rc_command.aux2;
    }

    // mode: 0=attitude, 1=speed, 2=position
    if (rc_values[CH_MODE] < RC_CENTER - 200) {
        rc_command.mode = 0;
    } else if (rc_values[CH_MODE] > RC_CENTER + 200) {
        rc_command.mode = 2;
    } else {
        rc_command.mode = 1;
    }

    if (rc_command.aux2 == 1) {
        // aux2=1: reset mode
        wheel_command = {0.0, 0.0, 0.0, 0.0};
    } 
    else if (rc_command.aux2 == 0) {
        if (abs(rc_values[0] - 1017.0) <= 10.0) {
            rc_command.lateral = 0.0;
        } else {
            rc_command.lateral = mapf(rc_values[0], 200.0, 1800.0, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        }
        if (abs(rc_values[1] - 1046.0) <= 10.0) {
            rc_command.pitch = 0.0;
        } else {
            rc_command.pitch = mapf(rc_values[1], 200.0, 1800.0, MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
        }
        rc_command.yaw = mapf(rc_values[2], 200.0, 1800.0, 0.0, MAX_MOTOR_SPEED);

        // aux2=0: normal 3 modes
        switch (rc_command.mode) {
            case 0: // attitude control
                // Use all channels for speed
                for (int i = 0; i < 4; i++) {
                    wheel_command[i] = (PWM_MAP_MATRIX[i][0] * rc_command.lateral +
                                       PWM_MAP_MATRIX[i][1] * rc_command.pitch +
                                       PWM_MAP_MATRIX[i][2] * rc_command.yaw) ;
                }
                break;
            case 1: // motor speed control
                for (int i = 0; i < 4; i++){
                    wheel_command[i] = PWM_MAP_MATRIX[i][0] * rc_command.lateral;
                }
                break;
            case 2: // position control
                // // (User to implement position logic)
                // for (int i = 0; i < 4; i++) {
                //     wheel_command[i] = 0.0;
                // }
                break;
        }
        // 應用低通濾波
        for(int i = 0; i < 4; i++){
            filtered_wheel_command[i] = low_pass_filter(wheel_command[i], filtered_wheel_command[i]);
            // 死區處理
            if (abs(filtered_wheel_command[i]) < 2.0) {
                filtered_wheel_command[i] = 0.0;
            }
        }

    } else if (rc_command.aux2 == 2) {
        // aux2=2: Segway control, 3 modes
        switch (rc_command.mode) {
            case 0: // Segway speed
                if (abs(rc_values[1] - 1046.0) <= 10.0) {
                    rc_command.pitch = 0.0;
                } else {
                    rc_command.pitch = mapf(rc_values[1], 200.0, 1800.0, MAX_PITCH_RATE, -MAX_PITCH_RATE);
                }
                SEG_PITCH_RATE_COMMAND = rc_command.pitch;
                FILTERED_SEG_PITCH_RATE_COMMAND = low_pass_filter(SEG_PITCH_RATE_COMMAND, FILTERED_SEG_PITCH_RATE_COMMAND);
                break;
            case 1: // Segway attitude
                // Use pitch for attitude
                if (abs(rc_values[1] - 1046.0) <= 10.0) {
                    rc_command.pitch = 0.0;
                } else {
                    rc_command.pitch = mapf(rc_values[1], 200.0, 1800.0, MAX_PITCH, -MAX_PITCH);
                }
                SEG_PITCH_COMMAND = rc_command.pitch;
                FILTERED_SEG_PITCH_COMMAND = low_pass_filter(SEG_PITCH_COMMAND, FILTERED_SEG_PITCH_COMMAND);
                break;
            case 2: // Segway position
                break;
        }
    }
}

void Segway_Pitch_Rate_controller(double FILTERED_SEG_PITCH_RATE_COMMAND){
    double pitch_rate = Segway_ang_vel[1];
    
    SEG_PITCH_RATE_ERROR = FILTERED_SEG_PITCH_RATE_COMMAND - pitch_rate;
    
    if(sign(FILTERED_SEG_PITCH_RATE_COMMAND) != sign(PRE_SEG_PITCH_RATE_COMMAND)){
        SEG_PITCH_RATE_ERROR_INTEGRAL = 0.0;
    }

    SEG_PITCH_RATE_ERROR_INTEGRAL += SEG_PITCH_RATE_ERROR * T;
    SEG_PITCH_RATE_ERROR_DERIVATIVE = (SEG_PITCH_ERROR - SEG_PITCH_ERROR_PREV) / T;
       
    double KP = SEG_PITCH_RATE_Kp * SEG_PITCH_RATE_ERROR;
    double KI = SEG_PITCH_RATE_Ki * SEG_PITCH_RATE_ERROR_INTEGRAL;
    double KD = SEG_PITCH_RATE_Kd * SEG_PITCH_RATE_ERROR_DERIVATIVE;

    if(abs(KI) >= 0.8 * MAX_MOTOR_SPEED){
        KI = 0.8 * MAX_MOTOR_SPEED * sign(KI);
    }

    SEG_CONTROL_LAW[1] = KP + KI + KD;
    if(abs(SEG_CONTROL_LAW[1]) >= MAX_MOTOR_SPEED){
        SEG_CONTROL_LAW[1] = MAX_MOTOR_SPEED * sign(SEG_CONTROL_LAW[1]);
    }
    SEG_PITCH_RATE_ERROR_PREV = SEG_PITCH_RATE_ERROR;

    for(int i = 0; i < 4; i++){
        wheel_command[i] = (PWM_MAP_MATRIX[i][0] * SEG_CONTROL_LAW[0] +
                            PWM_MAP_MATRIX[i][1] * SEG_CONTROL_LAW[1] +
                            PWM_MAP_MATRIX[i][2] * SEG_CONTROL_LAW[2]);

        filtered_wheel_command[i] = low_pass_filter(wheel_command[i], filtered_wheel_command[i]);
    }
}

void Segway_Pitch_controller(double FILTERED_SEG_PITCH_COMMAND){
    double pitch = attitude[1];

    SEG_PITCH_ERROR = FILTERED_SEG_PITCH_COMMAND - pitch;
    SEG_PITCH_ERROR = SEG_PITCH_ERROR * PI / 180.0;
    
    if(sign(FILTERED_SEG_PITCH_COMMAND) != sign(PRE_SEG_PITCH_COMMAND)){
        SEG_PITCH_ERROR_INTEGRAL = 0.0;
    }

    SEG_PITCH_ERROR_INTEGRAL += SEG_PITCH_ERROR * T;
    SEG_PITCH_ERROR_DERIVATIVE = (SEG_PITCH_ERROR - SEG_PITCH_ERROR_PREV) / T;
    
    
    double KP = SEG_PITCH_Kp * SEG_PITCH_ERROR;
    double KI = SEG_PITCH_Ki * SEG_PITCH_ERROR_INTEGRAL;
    double KD = SEG_PITCH_Kd * SEG_PITCH_ERROR_DERIVATIVE;

    if(abs(KI) >= 0.5 * MAX_PITCH_RATE){
        KI = 0.5 * MAX_PITCH_RATE * sign(KI);
    }

    SEG_PITCH_TO_PITCH_RATE_CMD = KP + KI + KD;
    if(abs(SEG_PITCH_TO_PITCH_RATE_CMD) >= MAX_PITCH_RATE){
        SEG_PITCH_TO_PITCH_RATE_CMD = MAX_PITCH_RATE * sign(SEG_PITCH_TO_PITCH_RATE_CMD);
    }

    FILTERED_SEG_PITCH_TO_PITCH_RATE_CMD = low_pass_filter(SEG_PITCH_TO_PITCH_RATE_CMD, FILTERED_SEG_PITCH_TO_PITCH_RATE_CMD);
    SEG_PITCH_ERROR_PREV = SEG_PITCH_ERROR;
}

void motor_controller(array<double, 4>& filtered_wheel_command) {
    for (int j = 0; j < 4; j++) {
        // Calculate error terms
        e[j] = filtered_wheel_command[j] - filtered_wheel_ang_vel[j];
        
        // Reset integral when target speed sign changes
        if (sign(filtered_wheel_command[j]) != sign(pre_wheel_command[j])) {
            e_integral[j] = 0.0;
        }
        
        e_integral[j] += e[j] * T;
        e_derivative[j] = (e[j] - e_prev[j]) / T;
        
        // Calculate PID terms
        double KP = MOTOR_Kp[j] * e[j];
        double KI = MOTOR_Ki[j] * e_integral[j];
        double KD = MOTOR_Kd[j] * e_derivative[j];
        
        // Anti-windup for integral term
        if (abs(KI) >= 0.8 * MAX_PWM) {
            KI = 0.8 * MAX_PWM * sign(KI);
        }
        
        // Calculate motor power with direction
        motor_power[j] = KP + KI + KD;
        
        // Update previous error
        e_prev[j] = e[j];
    }
}

void attitude_init() {
    imu_data_t imu_temp;
    int idx = 0;
    SimpleTimer bias_timer;
    bias_timer.begin();
    while (bias_timer.elapsed() < 5000000) { // 5秒累積平均
        imu_mpu9250_update(mpu9250, imu_raw.gyro, imu_raw.accel, imu_raw.mag);
        for (int i = 0; i < 3; i++) {
            imu_temp.accel[i] += imu_raw.accel[i];
            imu_temp.gyro[i] += imu_raw.gyro[i];
            imu_temp.mag[i] += imu_raw.mag[i];
        }
        idx++;
        delay(1);
    }
    for (int i = 0; i < 3; i++) {
        imu_temp.accel[i] /= (double)(idx); // 平均加速度
        imu_temp.gyro[i] /= (double)(idx);  // 平均陀螺儀
        imu_temp.mag[i] /= (double)(idx);   // 平均磁力計
    }
    orient.init(T_AHRS, imu_temp.accel, imu_temp.mag, imu_temp.gyro); // 初始化姿態解算器
}


void print_data(){

    for(int i = 0; i < 3; i++){
        Serial.print(attitude[i] * PI / 180.0);
        Serial.print(" ");
    }

    for(int i = 0; i < 3; i++){
        Serial.print(Segway_ang_vel[i]);
        Serial.print(" ");
    }

    // Serial.print(rc_command.lateral, 3);
    // Serial.print(" ");
    Serial.print(rc_command.pitch * PI / 180.0, 3);
    Serial.print(" ");
    // Serial.print(rc_command.yaw, 3);
    // Serial.print(" ");

    Serial.print(SEG_PITCH_ERROR);
    Serial.print(" ");
    Serial.print(SEG_PITCH_RATE_ERROR_INTEGRAL);
    Serial.print(" ");
    Serial.print(FILTERED_SEG_PITCH_TO_PITCH_RATE_CMD);
    Serial.print(" ");

    Serial.print(SEG_PITCH_RATE_ERROR);
    Serial.print(" ");
    Serial.print(SEG_PITCH_RATE_ERROR_INTEGRAL);
    Serial.print(" ");

    for(int i = 0; i < 3; i++){
        Serial.print(SEG_CONTROL_LAW[i]);
        Serial.print(" ");
    }

    for(int i = 0; i < 4; i++){
        Serial.print(filtered_wheel_command[i], 3);
        Serial.print(" ");
        // Serial.print(motor_power[i]);
        // Serial.print(" ");
        // Serial.print(wheel_pos[i], 3);
        // Serial.print(" ");
        // Serial.print(wheel_ang_vel[i], 3);
        // Serial.print(" ");
        // Serial.print(filtered_wheel_ang_vel[i], 3);
        // Serial.print(" ");
    }
  
    Serial.println();
}
