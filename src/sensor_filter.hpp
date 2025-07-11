#ifndef SENSOR_FILTER_HPP
#define SENSOR_FILTER_HPP
#include <Arduino.h>
#include <ICM42688.h>
#include <MadgwickAHRS.h>

struct IMU_data {
    double att[3];
    double acc[3];
    double gyro[3];
    double gyro_rad[3];
    double mag[3];
    double quat[4];
};

double LPF(double raw, double filte, double cut_off_Hz, double dt) {
    double alpha = (2.0 * PI * cut_off_Hz * dt) / (1 + 2.0 * PI * cut_off_Hz * dt);
    filte = alpha * raw + (1 - alpha) * filte;
    return filte;
}

class LPF2_StateSpace {
public:
    LPF2_StateSpace(double sample_period, double cut_off_Hz, double damping) {
        T = sample_period;
        omega_n = 2.0f * PI * cut_off_Hz;
        zeta = damping;
        x1 = x2 = 0.0f;
    }
    double update(double u) {
        double x1_next = x1 + T * x2;
        double x2_next = x2 + T * (
            -2.0f * zeta * omega_n * x2
            - omega_n * omega_n * x1
            + omega_n * omega_n * u
            );
        x1 = x1_next;
        x2 = x2_next;
        return x1;
    }
    double x_dot() {
        return x2;
    }
    void reset(float val = 0.0f) {
        x1 = val;
        x2 = 0.0f;
    }
private:
    double T, omega_n, zeta;
    double x1, x2;
};

inline IMU_data get_imu(IMU_data imu, ICM42688& IMU) {
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    IMU.getAGT();
    SPI.endTransaction();
    imu.acc[0] = IMU.accX();
    imu.acc[1] = IMU.accY();
    imu.acc[2] = IMU.accZ();
    imu.gyro[0] = IMU.gyrX();
    imu.gyro[1] = IMU.gyrY();
    imu.gyro[2] = IMU.gyrZ();
    for (int i = 0; i < 3; i++) {
        imu.gyro_rad[i] = imu.gyro[i] / 57.29577;
    }
    return imu;
}

inline IMU_data update_AHRS(IMU_data imu, Madgwick& filter) {
    IMU_data output;
    filter.updateIMU(imu.gyro[0], imu.gyro[1], imu.gyro[2], imu.acc[0], imu.acc[1], imu.acc[2]);
    output.att[0] = filter.getRoll();
    output.att[1] = filter.getPitch();
    output.att[2] = filter.getYaw();
    return output;
}

#endif // SENSOR_FILTER_HPP 