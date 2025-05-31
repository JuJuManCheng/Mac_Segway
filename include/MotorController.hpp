#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "Motor.hpp"
#include <array>

/**
 * @brief MotorController class for managing multiple motors
 * 馬達控制器類別，用於管理多個馬達
 */
class MotorController {
private:
    static constexpr int NUM_MOTORS = 4;  // Number of motors / 馬達數量
    int max_pwm;                          // Maximum PWM value / 最大PWM值
    std::array<Motor, NUM_MOTORS> motors; // Array of motor objects / 馬達物件陣列

public:
    /**
     * @brief Constructor for MotorController class
     * 馬達控制器類別建構函數
     * 
     * @param dir_pins Array of direction control pins / 方向控制腳位陣列
     * @param pwm_pins Array of PWM control pins / PWM控制腳位陣列
     * @param reverse_flags Array of direction inversion flags / 方向反轉標誌陣列
     * @param pwm_resolution PWM resolution in bits / PWM解析度（位元）
     */
    MotorController(const std::array<uint8_t, NUM_MOTORS>& dir_pins,
                   const std::array<uint8_t, NUM_MOTORS>& pwm_pins,
                   const std::array<int, NUM_MOTORS>& reverse_flags,
                   int pwm_resolution = 12)
        : max_pwm((1 << pwm_resolution) - 1),
          motors{{
              Motor(dir_pins[0], pwm_pins[0], reverse_flags[0], max_pwm),
              Motor(dir_pins[1], pwm_pins[1], reverse_flags[1], max_pwm),
              Motor(dir_pins[2], pwm_pins[2], reverse_flags[2], max_pwm),
              Motor(dir_pins[3], pwm_pins[3], reverse_flags[3], max_pwm)
          }} {}

    /**
     * @brief Initialize all motors and PWM settings
     * 初始化所有馬達和PWM設定
     */
    void init() {
        analogWriteResolution(12);  // Set PWM resolution to 12 bits / 設定PWM解析度為12位元
        for (auto& motor : motors) {
            motor.init();
        }
    }

    /**
     * @brief Set power for all motors
     * 設定所有馬達的功率
     * 
     * @param power PWM value for all motors / 所有馬達的PWM值
     */
    void setAllPower(int power) {
        for (auto& motor : motors) {
            motor.setPower(power);
        }
    }

    /**
     * @brief Set power for a specific motor
     * 設定特定馬達的功率
     * 
     * @param index Motor index (0 to NUM_MOTORS-1) / 馬達索引 (0 到 NUM_MOTORS-1)
     * @param power PWM value / PWM值
     */
    void setPower(int index, int power) {
        if (index >= 0 && index < NUM_MOTORS) {
            motors[index].setPower(power);
        }
    }

    /**
     * @brief Stop all motors
     * 停止所有馬達
     */
    void stopAll() {
        for (auto& motor : motors) {
            motor.stop();
        }
    }

    /**
     * @brief Get maximum PWM value
     * 獲取最大PWM值
     * 
     * @return Maximum PWM value / 最大PWM值
     */
    int getMaxPWM() const {
        return max_pwm;
    }
};

#endif // MOTOR_CONTROLLER_HPP 