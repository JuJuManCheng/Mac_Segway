#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

/**
 * @brief Motor class for controlling a single DC motor
 * 馬達類別，用於控制單個直流馬達
 */
class Motor {
private:
    uint8_t dir_pin;    // Direction control pin / 方向控制腳位
    uint8_t pwm_pin;    // PWM control pin / PWM控制腳位
    int reverse_flag;   // Direction inversion flag / 方向反轉標誌
    int max_pwm_value;  // Maximum PWM value / 最大PWM值

public:
    /**
     * @brief Default constructor
     * 預設建構函數
     */
    Motor() : dir_pin(0), pwm_pin(0), reverse_flag(1), max_pwm_value(4095) {}

    /**
     * @brief Constructor for Motor class
     * 馬達類別建構函數
     * 
     * @param dir_pin Direction control pin number / 方向控制腳位編號
     * @param pwm_pin PWM control pin number / PWM控制腳位編號
     * @param reverse_flag Direction inversion flag (1 or -1) / 方向反轉標誌 (1 或 -1)
     * @param max_pwm_value Maximum PWM value / 最大PWM值
     */
    Motor(uint8_t dir_pin, uint8_t pwm_pin, int reverse_flag = 1, int max_pwm_value = 4095)
        : dir_pin(dir_pin), pwm_pin(pwm_pin), reverse_flag(reverse_flag), max_pwm_value(max_pwm_value) {}

    /**
     * @brief Initialize the motor
     * 初始化馬達
     */
    void init() {
        pinMode(dir_pin, OUTPUT);
        pinMode(pwm_pin, OUTPUT);
        analogWriteFrequency(pwm_pin, 30000);  // Set PWM frequency to 30kHz
        stop();
    }

    /**
     * @brief Set motor power
     * 設定馬達功率
     * 
     * @param power PWM value (-max_pwm_value to max_pwm_value) / PWM值 (-max_pwm_value 到 max_pwm_value)
     */
    void setPower(int power) {
        // Apply direction inversion
        power = reverse_flag * power;

        if (power > 0) {
            digitalWrite(dir_pin, HIGH);
            analogWrite(pwm_pin, max_pwm_value - power);
        } else if (power < 0) {
            digitalWrite(dir_pin, LOW);
            analogWrite(pwm_pin, max_pwm_value + power);
        } else {
            stop();
        }
    }

    /**
     * @brief Stop the motor
     * 停止馬達
     */
    void stop() {
        digitalWrite(dir_pin, LOW);
        analogWrite(pwm_pin, max_pwm_value);
    }
};

#endif // MOTOR_HPP 