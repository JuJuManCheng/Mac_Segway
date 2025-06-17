#ifndef IMU_MPU9255_UTILITY_HPP
#define IMU_MPU9255_UTILITY_HPP

#include <Arduino.h>
#include <MPU9250.h>

// Initialize MPU9250
void imu_mpu9250_init(MPU9250& mpu);

// Update IMU data
void imu_mpu9250_update(MPU9250& mpu, double* gyro, double* accel, double* mag);

#endif // IMU_MPU9255_UTILITY_HPP 