#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <cmath>
#include <sbus.h> // SBUS 接收器函式庫
#include <LS7366R.h>
#include "MotorController.hpp"
#include <iostream>
#include <vector>
#include <simple_timer.hpp>
#include <ICM42688.h>
#include <MadgwickAHRS.h>
#include "sensor_filter.hpp"
#include "third_order_cpf.hpp"

// #include "imu_mpu9255_utility.hpp"
// #include "imu_interface.hpp"
// #include <MPU9250.h>

using namespace std;

// Hardware interface definitions
#define HW_SERIAL_INTERFACE_SBUS Serial1
#define count_perRev 1400.0

#define voltage_pin A3
double voltage = 0.0;

// ===== Pin Configurations =====
const array<uint8_t, 4> PIN_DIR = {15, 14, 6, 28};    // Direction pins
const array<uint8_t, 4> PIN_PWM = {25, 24, 9, 29};    // PWM pins
const array<int, 4> PWM_REVERSE = {-1, 1, -1, 1};
const array<array<double, 3>, 4> MAP_MATRIX = {{
    array<double, 3>{1.0, 1.0, 1.0},
    array<double, 3>{1.0, -0.4, 0.4},
    array<double, 3>{1.0,  0.4, -0.4},
    array<double, 3>{1.0, -1.0, -1.0}
}};

const array<uint8_t, 4> PIN_QEI = {30, 31, 33, 32};   // Quadrature encoder pins

// ===== PWM Configuration =====
const int CONFIG_ANALOG_WRITE_FREQ = 30000;                // PWM frequency in Hz
const int CONFIG_ANALOG_WRITE_RES = 12;                    // PWM resolution in bits
const int MAX_PWM = (int)(pow(2, CONFIG_ANALOG_WRITE_RES)) - 1;  // Maximum PWM value

// ===== Madgwick AHRS & Orientation Integration =====
#define AHRS_UPDATE_PERIOD_MICROS 1000
#define IMU_UPDATE_PERIOD_MICROS 250
#define ENCODER_UPDATE_PERIOD_MICROS 200

double T_AHRS = (double)(AHRS_UPDATE_PERIOD_MICROS) / 1000000.0;
double HZ = 1.0 / T_AHRS;
double T_IMU = (double)(IMU_UPDATE_PERIOD_MICROS) / 1000000.0;
double IMU_LPF_CUT_HZ = 40;

struct Timers {
    SimpleTimer ahrs;
    SimpleTimer imu;
    SimpleTimer encoder;
} timers;

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

// ===== Global Objects =====
ICM42688 IMU(SPI, 10);
MotorController motorController(PIN_DIR, PIN_PWM, PWM_REVERSE, CONFIG_ANALOG_WRITE_RES);
LS7366R qei;
IEC::SBUS sbus(HW_SERIAL_INTERFACE_SBUS);

Madgwick filter;
LPF2_StateSpace angle_filter_x(T_IMU, IMU_LPF_CUT_HZ, 1.0);
LPF2_StateSpace angle_filter_y(T_IMU, IMU_LPF_CUT_HZ, 1.0);

double wx = 0.0, wx_dot = 0.0, wy = 0.0, wy_dot = 0.0;
double filtered_wx = 0.0, filtered_wx_dot = 0.0, filtered_wy = 0.0, filtered_wy_dot = 0.0;

IMU_data raw_data;
IMU_data filter_data;
IMU_data AHRS_data;

RC_Command rc_command;

IntervalTimer timer;

// ===== Attitude =====
array<double, 3> attitude = {0.0, 0.0, 0.0}; // attitude[0]=roll, [1]=pitch, [2]=yaw
array<double, 3> Segway_ang_vel = {0.0, 0.0, 0.0}; // Segway_ang_vel[0]=roll rate, [1]=pitch rate, [2]=yaw rate
array<double, 3> Filtered_Segway_ang_vel = {0.0, 0.0, 0.0}; // Segway_ang_acc[0]=roll acc, [1]=pitch acc, [2]=yaw acc
array<double, 3> Segway_accel = {0.0, 0.0, 0.0}; // Segway_accel[0]=x acc, [1]=y acc, [2]=z acc
array<double, 3> Segway_ang_acc = {0.0, 0.0, 0.0}; // Segway_ang_acc[0]=roll acc, [1]=pitch acc, [2]=yaw acc
array<double, 3> Filtered_Segway_ang_acc = {0.0, 0.0, 0.0}; // Segway_ang_acc[0]=roll acc, [1]=pitch acc, [2]=yaw acc

// ===== RC Input Configuration =====
const uint16_t RC_MIN = 200;                               // Minimum RC input value
const uint16_t RC_MAX = 1800;                              // Maximum RC input value
const uint16_t RC_CENTER = 1000;                           // Center RC input value
const double MAX_MOTOR_SPEED = 30.0;                             // Maximum speed in rad/s
const double MAX_PITCH = 2.0;                             // Maximum pitch 
const double MAX_PITCH_RATE = 5.0;                         // rad/s
const double MAX_Vx = 2.0;                                 // Max forward speed V_x
const double MAX_Vy = 5.0;                                 // Max lateral speed V_y
const double Max_Wz = 1.0;

// ===== Control Loop Configuration =====
const double T = 0.005;                                      
const double rc_fc = 50;
const double RC_FILTER_TAU = 1 / (2*PI*rc_fc);             // time constant for RC filtering
const double RC_FILTER_Alpha = exp(-T / RC_FILTER_TAU);    // Filter coefficient
const double vel_fc = 40;
const double VEL_FILTER_TAU = 1 / (2*PI*vel_fc);             // time constant for RC filtering
const double VEL_FILTER_Alpha = exp(-T / VEL_FILTER_TAU);    // Filter coefficient

LPF2_StateSpace PITCH_RATE_CPF(T, 10, 1.0);
LPF3_StateSpace PITCH_CPF(T, 10, 1.0);

// RC for Segway Control
double SEG_Px_COMMAND = 0.0;
double PRE_SEG_Px_COMMAND = 0.0;

double SEG_Vx_COMMAND = 0.0;
double PRE_SEG_Vx_COMMAND = 0.0;
double FILTERED_SEG_Vx_COMMAND = 0.0;

double SEG_Vy_COMMAND = 0.0;
double PRE_SEG_Vy_COMMAND = 0.0;
double FILTERED_SEG_Vy_COMMAND = 0.0;

double SEG_Wz_COMMAND = 0.0;
double PRE_SEG_Wz_COMMAND = 0.0;
double FILTERED_SEG_Wz_COMMAND = 0.0;

double SEG_PITCH_COMMAND = 0.0;
double PRE_SEG_PITCH_COMMAND = 0.0;
double FILTERED_SEG_PITCH_COMMAND = 0.0;
double FILTERED_SEG_PITCH_COMMAND_DOT = 0.0;
double FILTERED_SEG_PITCH_COMMAND_DDOT = 0.0;
double NEW_FILTERED_SEG_PITCH_COMMAND_DOT = 0.0;
double NEW_FILTERED_SEG_PITCH_COMMAND_DDOT = 0.0;

double SEG_PITCH_RATE_COMMAND = 0.0;
double PRE_SEG_PITCH_RATE_COMMAND = 0.0;
double FILTERED_SEG_PITCH_RATE_COMMAND = 0.0;
double FILTERED_SEG_PITCH_RATE_COMMAND_DOT = 0.0;

double SEG_Vx_TO_PITCH_CMD = 0.0;
double FILTERED_SEG_Vx_TO_PITCH_CMD = 0.0;
double PRE_SEG_Vx_TO_PITCH_CMD = 0.0;

// ===== MotorPID Configuration =====
const array<double, 4> MOTOR_Kp = {500.0, 500.0, 500.0, 500.0};  // Proportional gains
const array<double, 4> MOTOR_Ki = {2000.0, 2000.0, 2000.0, 2000.0};      // Integral gains
const array<double, 4> MOTOR_Kd = {0.0, 0.0, 0.0, 0.0};          // Derivative gains

// ===== Segway PID Configuration =====
const double SEG_Px_Kp = 0.01;
const double SEG_Px_Ki = 0.0;      
const double SEG_Px_Kd = 0.1;
double SEG_Px_ERROR = 0.0;
double SEG_Px_ERROR_INTEGRAL = 0.0;
double SEG_Px_ERROR_DERIVATIVE = 0.0;
double FILTERED_SEG_Px_ERROR_DERIVATIVE = 0.0;
double SEG_Px_ERROR_PREV = 0.0;

const double SEG_Vx_Kp = 0.5;
const double SEG_Vx_Ki = 0.0;      
const double SEG_Vx_Kd = 0.0;
double SEG_Vx_ERROR = 0.0;
double SEG_Vx_ERROR_INTEGRAL = 0.0;
double SEG_Vx_ERROR_DERIVATIVE = 0.0;
double FILTERED_SEG_Vx_ERROR_DERIVATIVE = 0.0;
double SEG_Vx_ERROR_PREV = 0.0;

const double SEG_PITCH_FFG = -0.0;
const double SEG_PITCH_Kp = -180.0;
const double SEG_PITCH_Ki = -30.0;    
const double SEG_PITCH_Kd1 = -4.0;   
const double SEG_PITCH_Kd2 = -0.03; 
// const double SEG_PITCH_Kd1 = -4.0;   
// const double SEG_PITCH_Kd2 = -0.02;     
double SEG_PITCH_ERROR = 0.0;
double SEG_PITCH_ERROR_INTEGRAL = 0.0;
double SEG_PITCH_ERROR_DERIVATIVE = 0.0;
double SEG_PITCH_ERROR_DDERIVATIVE = 0.0;
double SEG_PITCH_ERROR_PREV = 0.0;

array<double, 3> SEG_CONTROL_LAW = {0.0, 0.0, 0.0};

bool is_stop = false;

// RC for motor Control
uint16_t rc[16] = {0};
array<double, 4> wheel_command = {0.0, 0.0, 0.0, 0.0};
array<double, 4> pre_wheel_command = {0.0, 0.0, 0.0, 0.0};
array<double, 4> filtered_wheel_command = {0.0, 0.0, 0.0, 0.0};
static int last_aux2 = -1; // 初始狀態為 -1 表示第一次執行
static int last_mode = -1;

// Motor Control
array<double, 4> motor_power = {0.0, 0.0, 0.0, 0.0};

// Encoder Data
double cur_position = 0.0;
array<double, 4> wheel_pos = {0.0, 0.0, 0.0, 0.0};
array<double, 4> wheel_last_pos = {0.0, 0.0, 0.0, 0.0};
array<double, 4> wheel_ang_vel = {0.0, 0.0, 0.0, 0.0};
array<double, 4> wheel_ang_vel_prev = {0.0, 0.0, 0.0, 0.0};
array<double, 4> filtered_wheel_ang_vel = {0.0, 0.0, 0.0, 0.0};

array<double, 3> Segway_pos = {0.0, 0.0, 0.0};       // Px, Py, THETA_z
array<double, 3> Segway_last_pos = {0.0, 0.0, 0.0};

array<double, 3> Segway_vel = {0.0, 0.0, 0.0};      // Vx, Vy, Wz
array<double, 3> Segway_last_vel = {0.0, 0.0, 0.0};
array<double, 3> filtered_Segway_vel = {0.0, 0.0, 0.0};

// Motor PID Variables
array<double, 4> e = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_integral = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_derivative = {0.0, 0.0, 0.0, 0.0};
array<double, 4> e_prev = {0.0, 0.0, 0.0, 0.0};

// ===== Helper Functions =====
void reset();
void routine();
void print_data();
double sign(double x);
double low_pass_filter(double new_value, double pre_val, double alpha);
double mapf(int x, double in_min, double in_max, double out_min, double out_max);

void get_encoder();
void RC_Process_command(uint16_t* rc_values);
void get_vel(array<double, 4>& wheel_pos, array<double, 4>& wheel_last_pos, array<double, 4>& wheel_ang_vel);
void get_sensor_data();

void Segway_Px_controller(double SEG_Px_COMMAND);
void Segway_Vx_controller(double FILTERED_SEG_Vx_COMMAND);
void Segway_Pitch_controller(double FILTERED_SEG_PITCH_COMMAND, double NEW_FILTERED_SEG_PITCH_COMMAND_DOT, double NEW_FILTERED_SEG_PITCH_COMMAND_DDOT);
void motor_controller(array<double, 4>& filtered_wheel_command);

// ===== Setup and Loop =====
void setup() {
    Serial.begin(256000);
    while (IMU.begin() < 0) {
        int status = IMU.begin();
        delay(1000);
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
    }
    IMU.setAccelODR(ICM42688::odr4k);
    IMU.setGyroODR(ICM42688::odr4k);
    IMU.setAccelFS(ICM42688::gpm8);
    IMU.setGyroFS(ICM42688::dps2000);
    filter.begin(HZ);
    timers.ahrs.begin();
    timers.imu.begin();
    timers.encoder.begin();
    sbus.begin();                       // 初始化SBUS
    qei.begin(PIN_QEI.data(), 4);       // 初始化編碼器
    pinMode(voltage_pin, INPUT);        // 初始化電壓腳位
    analogReadResolution(12);           // 設定類比讀取解析度為12位元
    motorController.init();             // 初始化馬達控制器
    motorController.stopAll();          // 停止所有馬達
    timer.begin(routine, T*1000000);    
    reset();                            // 重設控制變數
    Serial.println("System initialized!");
    Serial.println("Direction configuration:");
    delay(1000);
}

void loop() {

    if (timers.imu.elapsed() >= IMU_UPDATE_PERIOD_MICROS) {
        timers.imu.begin();
        raw_data = get_imu(raw_data, IMU);
        for (int i = 0; i < 3; i++) {
            filter_data.acc[i] = LPF(raw_data.acc[i], filter_data.acc[i], IMU_LPF_CUT_HZ, T_IMU);
            filter_data.gyro[i] = LPF(raw_data.gyro[i], filter_data.gyro[i], IMU_LPF_CUT_HZ, T_IMU);
            filter_data.gyro_rad[i] = LPF(raw_data.gyro_rad[i], filter_data.gyro_rad[i], IMU_LPF_CUT_HZ, T_IMU);
        }
        wx = angle_filter_x.update(raw_data.gyro[0]);
        filtered_wx = low_pass_filter(wx, filtered_wx, RC_FILTER_Alpha);
        
        wx_dot = angle_filter_x.x_dot();
        filtered_wx_dot = low_pass_filter(wx_dot, filtered_wx_dot, RC_FILTER_Alpha);
        
        wy = angle_filter_y.update(raw_data.gyro[1]);
        filtered_wy = low_pass_filter(wy, filtered_wy, RC_FILTER_Alpha);
        
        wy_dot = angle_filter_y.x_dot();
        filtered_wy_dot = low_pass_filter(wy_dot, filtered_wy_dot, 0.2);
    }

    if (timers.ahrs.elapsed() >= AHRS_UPDATE_PERIOD_MICROS) {
        timers.ahrs.begin();
        AHRS_data = update_AHRS(filter_data, filter);
    }

    get_encoder();

    // 讀取SBUS資料
    sbus.read(rc);
    voltage = 0.0078 * (double)(analogRead(voltage_pin)); // 類比電壓轉換

    // 更新馬達輸出
    for (int i = 0; i < 4; i++) {
        motorController.setPower(i, motor_power[i]);
    }
}

void routine() {
    // Process RC command
    RC_Process_command(rc);

    get_sensor_data();

    if(abs(attitude[1]) >= 50.0){
        motor_power = {0.0, 0.0, 0.0, 0.0};
        reset();
    }

    if (rc_command.aux2 == 2) {  // 使用 aux2 作為啟動開關
        switch (rc_command.mode) {
            case 1:
                Segway_Px_controller(SEG_Px_COMMAND);
                PRE_SEG_Px_COMMAND = SEG_Px_COMMAND;

                Segway_Pitch_controller(FILTERED_SEG_PITCH_COMMAND, NEW_FILTERED_SEG_PITCH_COMMAND_DOT, NEW_FILTERED_SEG_PITCH_COMMAND_DDOT);
                PRE_SEG_PITCH_COMMAND = FILTERED_SEG_PITCH_COMMAND;
                
                motor_controller(filtered_wheel_command);
                pre_wheel_command = filtered_wheel_command;
                
                break;
            case 0:  // ATTITUDE CONTROL (PITCH)
                Segway_Pitch_controller(FILTERED_SEG_PITCH_COMMAND, NEW_FILTERED_SEG_PITCH_COMMAND_DOT, NEW_FILTERED_SEG_PITCH_COMMAND_DDOT);
                PRE_SEG_PITCH_COMMAND = FILTERED_SEG_PITCH_COMMAND;
                
                motor_controller(filtered_wheel_command);
                pre_wheel_command = filtered_wheel_command;
                break;
            case 2:  // position control
                if(rc_command.pitch == 0){
                    
                    Segway_Px_controller(SEG_Px_COMMAND);
                    PRE_SEG_Px_COMMAND = SEG_Px_COMMAND;

                    Segway_Pitch_controller(FILTERED_SEG_PITCH_COMMAND, NEW_FILTERED_SEG_PITCH_COMMAND_DOT, NEW_FILTERED_SEG_PITCH_COMMAND_DDOT);
                    PRE_SEG_PITCH_COMMAND = FILTERED_SEG_PITCH_COMMAND;
                
                    motor_controller(filtered_wheel_command);
                    pre_wheel_command = filtered_wheel_command;
                    break;
                }
                else{

                    Segway_Vx_controller(FILTERED_SEG_Vx_COMMAND);
                    PRE_SEG_Vx_COMMAND = FILTERED_SEG_Vx_COMMAND;
                    
                    Segway_Pitch_controller(FILTERED_SEG_PITCH_COMMAND, FILTERED_SEG_PITCH_COMMAND_DOT, FILTERED_SEG_PITCH_COMMAND_DDOT);
                    PRE_SEG_PITCH_COMMAND = FILTERED_SEG_PITCH_COMMAND;
                    
                    motor_controller(filtered_wheel_command);
                    pre_wheel_command = filtered_wheel_command;
                    break;
                }
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
                motor_power = {MAX_PWM, 0.0, 0.0, 0.0};
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

    SEG_Px_COMMAND = 0.0;
    PRE_SEG_Px_COMMAND = 0.0;

    SEG_Vx_COMMAND = 0.0;
    PRE_SEG_Vx_COMMAND = 0.0;
    FILTERED_SEG_Vx_COMMAND = 0.0;

    SEG_Vy_COMMAND = 0.0;
    PRE_SEG_Vy_COMMAND = 0.0;
    FILTERED_SEG_Vy_COMMAND = 0.0;

    SEG_Wz_COMMAND = 0.0;
    PRE_SEG_Wz_COMMAND = 0.0;
    FILTERED_SEG_Wz_COMMAND = 0.0;

    SEG_PITCH_COMMAND = 0.0;
    PRE_SEG_PITCH_COMMAND = 0.0;
    FILTERED_SEG_PITCH_COMMAND = 0.0;
    
    FILTERED_SEG_PITCH_COMMAND_DOT = 0.0;
    FILTERED_SEG_PITCH_COMMAND_DDOT = 0.0;
    NEW_FILTERED_SEG_PITCH_COMMAND_DOT = 0.0;
    NEW_FILTERED_SEG_PITCH_COMMAND_DDOT = 0.0;

    SEG_PITCH_RATE_COMMAND = 0.0;
    PRE_SEG_PITCH_RATE_COMMAND = 0.0;
    FILTERED_SEG_PITCH_RATE_COMMAND = 0.0;
    FILTERED_SEG_PITCH_RATE_COMMAND_DOT = 0.0;

    SEG_Vx_TO_PITCH_CMD = 0.0;
    FILTERED_SEG_Vx_TO_PITCH_CMD = 0.0;
    PRE_SEG_Vx_TO_PITCH_CMD = 0.0;

    SEG_Px_ERROR = 0.0;
    SEG_Px_ERROR_INTEGRAL = 0.0;
    SEG_Px_ERROR_DERIVATIVE = 0.0;
    FILTERED_SEG_Px_ERROR_DERIVATIVE = 0.0;
    SEG_Px_ERROR_PREV = 0.0;

    SEG_Vx_ERROR = 0.0;
    SEG_Vx_ERROR_INTEGRAL = 0.0;
    SEG_Vx_ERROR_DERIVATIVE = 0.0;
    FILTERED_SEG_Vx_ERROR_DERIVATIVE = 0.0;
    SEG_Vx_ERROR_PREV = 0.0;

    SEG_PITCH_ERROR = 0.0;
    SEG_PITCH_ERROR_INTEGRAL = 0.0;
    SEG_PITCH_ERROR_DERIVATIVE = 0.0;
    SEG_PITCH_ERROR_DDERIVATIVE = 0.0;
    SEG_PITCH_ERROR_PREV = 0.0;

    e.fill(0.0);
    e_integral.fill(0.0);
    e_derivative.fill(0.0);
    e_prev.fill(0.0);

    SEG_CONTROL_LAW.fill(0.0);
    is_stop = false;
}

double low_pass_filter(double new_value, double pre_val, double alpha){
    double filtered_val = alpha * new_value + (1 - alpha) * pre_val;
    return filtered_val;
}

void get_sensor_data() {
    get_vel(wheel_pos, wheel_last_pos, wheel_ang_vel); // Wheel angular velocity
    attitude[0] = AHRS_data.att[0]; // roll
    attitude[1] = AHRS_data.att[1]; // pitch
    attitude[2] = AHRS_data.att[2]; // yaw
    // IMU data
    Segway_accel[0] = filter_data.acc[0]; // x acc
    Segway_accel[1] = filter_data.acc[1]; // y acc
    Segway_accel[2] = filter_data.acc[2]; // z acc

    // Segway_ang_vel[0] = filter_data.gyro[0]; // roll rate
    // Segway_ang_vel[1] = filter_data.gyro[1]; // pitch rate
    Segway_ang_vel[0] = wx; // roll rate
    Segway_ang_vel[1] = wy; // pitch rate
    Filtered_Segway_ang_vel[0] = filtered_wx; // roll rate
    Filtered_Segway_ang_vel[1] = filtered_wy; // pitch rate

    Segway_ang_acc[0] = wx_dot; // roll angular acceleration (二階濾波)
    Segway_ang_acc[1] = wy_dot; // pitch angular acceleration (二階濾波)
    Filtered_Segway_ang_acc[0] = filtered_wx_dot; // roll angular acceleration (二階濾波)
    Filtered_Segway_ang_acc[1] = filtered_wy_dot; // pitch angular acceleration (二階濾波)
}

void get_encoder(){

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    qei.read();
    SPI.endTransaction();

    cur_position = 0.0;
    for (int i = 0; i < 4; i++) {
        wheel_pos[i] = (i % 2 == 0) ? qei.get_pulse()[i] : -qei.get_pulse()[i];
        cur_position += (wheel_pos[i] / 4.0);
    }
}

void get_vel(array<double, 4>& wheel_pos, array<double, 4>& wheel_last_pos, array<double, 4>& wheel_ang_vel) {
    for (int j = 0; j < 4; j++) {
        wheel_ang_vel[j] = (wheel_pos[j] - wheel_last_pos[j]) * 2.0 * PI / (T * count_perRev);
        filtered_wheel_ang_vel[j] = low_pass_filter(wheel_ang_vel[j], filtered_wheel_ang_vel[j], VEL_FILTER_Alpha);
        wheel_ang_vel_prev[j] = wheel_ang_vel[j];
        wheel_last_pos[j] = wheel_pos[j];
    }

    for(int i = 0; i < 3; i++){
        Segway_vel[i] = (MAP_MATRIX[i][0] * filtered_wheel_ang_vel[0] +
                         MAP_MATRIX[i][1] * filtered_wheel_ang_vel[1] +
                         MAP_MATRIX[i][2] * filtered_wheel_ang_vel[2] +
                         MAP_MATRIX[i][3] * filtered_wheel_ang_vel[3]) / 4.0;
        Segway_last_vel[i] = Segway_vel[i];
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

    if(last_mode != rc_command.mode){
        reset();
        last_mode = rc_command.mode;
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
        if (abs(rc_values[0] - RC_CENTER) <= 30.0) {
            rc_command.lateral = 0.0;
        } else {
            rc_command.lateral = mapf(rc_values[0], RC_MIN, RC_MAX, MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
        }

        if (abs(rc_values[1] - RC_CENTER) <= 30.0) {
            rc_command.pitch = 0.0;
        } else {
            rc_command.pitch = mapf(rc_values[1], RC_MIN, RC_MAX, MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
        }
        rc_command.yaw = mapf(rc_values[2], RC_MIN, RC_MAX, 0.0, MAX_MOTOR_SPEED);

        // aux2=0: normal 3 modes
        switch (rc_command.mode) {
            case 0: // attitude control
                // Use all channels for speed
                for (int i = 0; i < 4; i++) {
                    wheel_command[i] = (MAP_MATRIX[i][0] * rc_command.pitch +
                                        MAP_MATRIX[i][1] * rc_command.lateral +
                                        MAP_MATRIX[i][2] * rc_command.yaw) ;
                }
                break;
            case 1: // motor speed control
                for (int i = 0; i < 4; i++){
                    wheel_command[i] = MAP_MATRIX[i][0] * rc_command.pitch;
                }
                break;
            case 2: 
                wheel_command[0] = MAX_MOTOR_SPEED;
                break;
        }
        // 應用低通濾波
        for(int i = 0; i < 4; i++){
            filtered_wheel_command[i] = low_pass_filter(wheel_command[i], filtered_wheel_command[i], RC_FILTER_Alpha);
            // 死區處理
            if (abs(filtered_wheel_command[i]) < 2.0) {
                filtered_wheel_command[i] = 0.0;
            }
        }
    } else if (rc_command.aux2 == 2) {
        // aux2=2: Segway control, 3 modes
        switch (rc_command.mode) {
            case 1: 
                if(is_stop == false){
                        for(int i = 0; i < 4; i++){
                        SEG_Px_COMMAND += wheel_pos[i] / 4.0;
                        }
                        is_stop = true;
                    } else{
                        SEG_Px_COMMAND = PRE_SEG_Px_COMMAND;
                    }

                break;
            case 0: // Segway attitude
                // Use pitch for attitude
                if (abs(rc_values[1] - RC_CENTER) <= 30.0) {
                    rc_command.pitch = 0.0;
                } else {
                    rc_command.pitch = mapf(rc_values[1], RC_MIN, RC_MAX, MAX_PITCH, -MAX_PITCH);
                }
                SEG_PITCH_COMMAND = rc_command.pitch;

                FILTERED_SEG_PITCH_COMMAND = low_pass_filter(SEG_PITCH_COMMAND, FILTERED_SEG_PITCH_COMMAND, RC_FILTER_Alpha);

                FILTERED_SEG_PITCH_COMMAND = PITCH_CPF.update(SEG_PITCH_COMMAND);
                
                FILTERED_SEG_PITCH_COMMAND_DOT = PITCH_CPF.x_dot();
                NEW_FILTERED_SEG_PITCH_COMMAND_DOT = low_pass_filter(FILTERED_SEG_PITCH_COMMAND_DOT, NEW_FILTERED_SEG_PITCH_COMMAND_DOT, RC_FILTER_Alpha);

                FILTERED_SEG_PITCH_COMMAND_DDOT = PITCH_CPF.x_ddot();
                NEW_FILTERED_SEG_PITCH_COMMAND_DDOT = low_pass_filter(FILTERED_SEG_PITCH_COMMAND_DDOT, NEW_FILTERED_SEG_PITCH_COMMAND_DDOT, 0.1);

                break;
            case 2: // Segway position
                if(abs(rc_values[0] - RC_CENTER) <= 50.0){
                    rc_command.yaw = 0;
                } else{
                    rc_command.yaw = mapf(rc_values[0], RC_MIN, RC_MAX, -Max_Wz, Max_Wz);
                }

                if(abs(rc_values[2] - RC_CENTER) <= 500.0){
                    rc_command.lateral = 0;
                } else{
                    rc_command.lateral = mapf(rc_values[2], RC_MIN, RC_MAX, -MAX_Vy, MAX_Vy);
                }
                
                SEG_Vy_COMMAND = rc_command.lateral;
                FILTERED_SEG_Vy_COMMAND = low_pass_filter(SEG_Vy_COMMAND, FILTERED_SEG_Vy_COMMAND, RC_FILTER_Alpha);

                SEG_Wz_COMMAND = rc_command.yaw;
                FILTERED_SEG_Wz_COMMAND = low_pass_filter(SEG_Wz_COMMAND, FILTERED_SEG_Wz_COMMAND, RC_FILTER_Alpha);
                
                SEG_CONTROL_LAW[1] = FILTERED_SEG_Vy_COMMAND;
                SEG_CONTROL_LAW[2] = FILTERED_SEG_Wz_COMMAND;

                if (abs(rc_values[1] - RC_CENTER) <= 50.0) {
                    rc_command.pitch = 0.0;
                    if(is_stop == false){
                        for(int i = 0; i < 4; i++){
                        SEG_Px_COMMAND += wheel_pos[i] / 4.0;
                        }
                        is_stop = true;
                    } else{
                        SEG_Px_COMMAND = PRE_SEG_Px_COMMAND;
                    }

                } else {
                    rc_command.pitch = mapf(rc_values[1], RC_MIN, RC_MAX, MAX_Vx, -MAX_Vx);

                    SEG_Vx_COMMAND = rc_command.pitch;
                    FILTERED_SEG_Vx_COMMAND = low_pass_filter(SEG_Vx_COMMAND, FILTERED_SEG_Vx_COMMAND, RC_FILTER_Alpha);
                    
                    is_stop = false;
                    SEG_Px_COMMAND = 0.0;
                }
                break;
        }
    }
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

void Segway_Pitch_controller(double FILTERED_SEG_PITCH_COMMAND, double NEW_FILTERED_SEG_PITCH_COMMAND_DOT, double NEW_FILTERED_SEG_PITCH_COMMAND_DDOT){
    double pitch = attitude[1], pitch_rate = Filtered_Segway_ang_vel[1], pitch_acc = Filtered_Segway_ang_acc[1];

    SEG_PITCH_ERROR = FILTERED_SEG_PITCH_COMMAND - pitch;
    SEG_PITCH_ERROR = SEG_PITCH_ERROR * PI / 180.0;
    
    if(sign(FILTERED_SEG_PITCH_COMMAND) != sign(PRE_SEG_PITCH_COMMAND)){
        SEG_PITCH_ERROR_INTEGRAL = 0.0;
    }

    SEG_PITCH_ERROR_INTEGRAL += SEG_PITCH_ERROR * T;
    SEG_PITCH_ERROR_DERIVATIVE = (NEW_FILTERED_SEG_PITCH_COMMAND_DOT - pitch_rate) * PI / 180.0;
    SEG_PITCH_ERROR_DDERIVATIVE = (NEW_FILTERED_SEG_PITCH_COMMAND_DDOT - pitch_acc) * PI / 180.0;
    // SEG_PITCH_ERROR_DERIVATIVE = (0 - pitch_rate) * PI / 180.0;
    // SEG_PITCH_ERROR_DDERIVATIVE = (0 - pitch_acc) * PI / 180.0;

    double FF = SEG_PITCH_FFG * NEW_FILTERED_SEG_PITCH_COMMAND_DOT;

    double KP = SEG_PITCH_Kp * SEG_PITCH_ERROR;
    double KI = SEG_PITCH_Ki * SEG_PITCH_ERROR_INTEGRAL;
    double KD1 = SEG_PITCH_Kd1 * SEG_PITCH_ERROR_DERIVATIVE;
    double KD2 = SEG_PITCH_Kd2 * SEG_PITCH_ERROR_DDERIVATIVE;

    if(abs(KI) >= 0.8 * MAX_MOTOR_SPEED){
        KI = 0.8 * MAX_MOTOR_SPEED * sign(KI);
    }

    SEG_CONTROL_LAW[0] = KP + KI + KD1 + KD2 + FF;
    if(abs(SEG_CONTROL_LAW[0]) >= MAX_MOTOR_SPEED){
        SEG_CONTROL_LAW[0] = MAX_MOTOR_SPEED * sign(SEG_CONTROL_LAW[0]);
    }

    SEG_PITCH_ERROR_PREV = SEG_PITCH_ERROR;

    for(int i = 0; i < 4; i++){
        wheel_command[i] = (MAP_MATRIX[i][0] * SEG_CONTROL_LAW[0] +
                            MAP_MATRIX[i][1] * SEG_CONTROL_LAW[1] +
                            MAP_MATRIX[i][2] * SEG_CONTROL_LAW[2]);

        filtered_wheel_command[i] = low_pass_filter(wheel_command[i], filtered_wheel_command[i], VEL_FILTER_Alpha);
    }

}

void Segway_Px_controller(double SEG_Px_COMMAND){
    double cur_pos = 0.0;
    for(int i = 0; i < 4; i++){
        cur_pos += wheel_pos[i] / 4.0;
    }

    SEG_Px_ERROR = SEG_Px_COMMAND - cur_pos;
    if(sign(SEG_Px_COMMAND) != sign(PRE_SEG_Px_COMMAND)){
        SEG_Px_ERROR_INTEGRAL = 0.0;
    }

    SEG_Px_ERROR_INTEGRAL += SEG_Px_ERROR * T;
    SEG_Px_ERROR_DERIVATIVE = (SEG_Px_ERROR - SEG_Px_ERROR_PREV) / T;
    FILTERED_SEG_Px_ERROR_DERIVATIVE = low_pass_filter(SEG_Px_ERROR_DERIVATIVE, FILTERED_SEG_Px_ERROR_DERIVATIVE, 0.2);

    double KP = SEG_Px_Kp * SEG_Px_ERROR;
    double KI = SEG_Px_Ki * SEG_Px_ERROR_INTEGRAL;
    double KD = SEG_Px_Kd * FILTERED_SEG_Px_ERROR_DERIVATIVE;

    if(abs(KI) >= 0.5 * MAX_PITCH){
        KI = 0.5 * MAX_PITCH * sign(KI);
    }

    SEG_PITCH_COMMAND = KP + KI + KD;
    if(abs(SEG_PITCH_COMMAND) >= MAX_PITCH){
        SEG_PITCH_COMMAND = MAX_PITCH * sign(SEG_PITCH_COMMAND);
    }
    
    FILTERED_SEG_PITCH_COMMAND = PITCH_CPF.update(SEG_PITCH_COMMAND);
    
    FILTERED_SEG_PITCH_COMMAND_DOT = PITCH_CPF.x_dot();
    NEW_FILTERED_SEG_PITCH_COMMAND_DOT = low_pass_filter(FILTERED_SEG_PITCH_COMMAND_DOT, NEW_FILTERED_SEG_PITCH_COMMAND_DOT, 0.2);

    FILTERED_SEG_PITCH_COMMAND_DDOT = PITCH_CPF.x_ddot();
    NEW_FILTERED_SEG_PITCH_COMMAND_DDOT = low_pass_filter(FILTERED_SEG_PITCH_COMMAND_DDOT, NEW_FILTERED_SEG_PITCH_COMMAND_DDOT, 0.2);

    SEG_Px_ERROR_PREV = SEG_Px_ERROR;

};

void Segway_Vx_controller(double FILTERED_SEG_Vx_COMMAND){
    double Vx = Segway_vel[0];
    
    SEG_Vx_ERROR = FILTERED_SEG_Vx_COMMAND - Vx;
    if(sign(FILTERED_SEG_Vx_COMMAND) != sign(FILTERED_SEG_Vx_COMMAND)){
        FILTERED_SEG_Vx_COMMAND = 0.0;
    }

    SEG_Vx_ERROR_INTEGRAL += SEG_Vx_ERROR * T;
    SEG_Vx_ERROR_DERIVATIVE = (SEG_Vx_ERROR - SEG_Vx_ERROR_PREV) / T;
    FILTERED_SEG_Vx_ERROR_DERIVATIVE = low_pass_filter(SEG_Vx_ERROR_DERIVATIVE, FILTERED_SEG_Px_ERROR_DERIVATIVE, 0.2);

    double KP = SEG_Vx_Kp * SEG_Vx_ERROR;
    double KI = SEG_Vx_Ki * SEG_Vx_ERROR_INTEGRAL;
    double KD = SEG_Vx_Kd * FILTERED_SEG_Vx_ERROR_DERIVATIVE;

    if(abs(KI) >= 0.5 * MAX_PITCH){
        KI = 0.5 * MAX_PITCH * sign(KI);
    }

    SEG_PITCH_COMMAND = KP + KI + KD;
    if(abs(SEG_PITCH_COMMAND) >= MAX_PITCH){
        SEG_PITCH_COMMAND = MAX_PITCH * sign(SEG_PITCH_COMMAND);
    }
    
    FILTERED_SEG_PITCH_COMMAND = PITCH_CPF.update(SEG_PITCH_COMMAND);
    
    FILTERED_SEG_PITCH_COMMAND_DOT = PITCH_CPF.x_dot();
    NEW_FILTERED_SEG_PITCH_COMMAND_DOT = low_pass_filter(FILTERED_SEG_PITCH_COMMAND_DOT, NEW_FILTERED_SEG_PITCH_COMMAND_DOT, 0.2);

    FILTERED_SEG_PITCH_COMMAND_DDOT = PITCH_CPF.x_ddot();
    NEW_FILTERED_SEG_PITCH_COMMAND_DDOT = low_pass_filter(FILTERED_SEG_PITCH_COMMAND_DDOT, NEW_FILTERED_SEG_PITCH_COMMAND_DDOT, 0.2);

    SEG_Px_ERROR_PREV = SEG_Px_ERROR;
};

void print_data(){

    Serial.print(rc_command.lateral);
    Serial.print(" ");
    
    Serial.print(rc_command.pitch);
    Serial.print(" ");
    
    Serial.print(rc_command.yaw);
    Serial.print(" ");

    Serial.print(FILTERED_SEG_Vx_COMMAND);
    Serial.print(" ");
    Serial.print(Segway_vel[0]);
    Serial.print(" ");

    Serial.print(SEG_PITCH_COMMAND);
    Serial.print(" ");
    Serial.print(FILTERED_SEG_PITCH_COMMAND);
    Serial.print(" ");

    for(int i = 0; i < 3; i++){
        Serial.print(SEG_CONTROL_LAW[i]);
        Serial.print(" ");
    }
    Serial.println();
}
