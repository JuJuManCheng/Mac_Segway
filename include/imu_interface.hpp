#pragma once

#include "MPU9250.h"

// IMU interface functions
void imu_mpu9250_init(MPU9250& imu);
void imu_mpu9250_update(MPU9250& imu, double* accel, double* gyro, double* mag); 