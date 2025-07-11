#include "imu_mpu9255_utility.hpp"

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

    gyro[0] = (double)(-mpu.getGyroY_rads());
    gyro[1] = (double)(-mpu.getGyroX_rads());
    gyro[2] = (double)(-mpu.getGyroZ_rads());

    
    accel[0] = (double)(-mpu.getAccelY_mss());
    accel[1] = (double)(-mpu.getAccelX_mss());
    accel[2] = (double)(-mpu.getAccelZ_mss());

    mag[0] = (double)(-mpu.getMagY_uT());
    mag[1] = (double)(-mpu.getMagX_uT());
    mag[2] = (double)(-mpu.getMagZ_uT());
} 