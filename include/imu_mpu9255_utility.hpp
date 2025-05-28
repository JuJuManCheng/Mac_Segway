#ifndef IMU_MPU9255_UTILITY_HPP
#define IMU_MPU9255_UTILITY_HPP

#include <Arduino.h>
#include <MPU9250.h>

// Initialize MPU9250
void imu_mpu9250_init(MPU9250& mpu) {
    int status = mpu.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
    }
    
    // Set accelerometer range to ±8G
    mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    
    // Set gyroscope range to ±500DPS
    mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    
    // Set magnetometer range to ±16G
    mpu.setMagCalX(0.0f, 1.0f);
    mpu.setMagCalY(0.0f, 1.0f);
    mpu.setMagCalZ(0.0f, 1.0f);
}

// Update IMU data
void imu_mpu9250_update(MPU9250& mpu, double* gyro, double* accel, double* mag) {
    mpu.readSensor();
    
    // Update accelerometer data
    accel[0] = mpu.getAccelX_mss();
    accel[1] = mpu.getAccelY_mss();
    accel[2] = mpu.getAccelZ_mss();
    
    // Update gyroscope data
    gyro[0] = mpu.getGyroX_rads();
    gyro[1] = mpu.getGyroY_rads();
    gyro[2] = mpu.getGyroZ_rads();
    
    // Update magnetometer data
    mag[0] = mpu.getMagX_uT();
    mag[1] = mpu.getMagY_uT();
    mag[2] = mpu.getMagZ_uT();
}

#endif // IMU_MPU9255_UTILITY_HPP 