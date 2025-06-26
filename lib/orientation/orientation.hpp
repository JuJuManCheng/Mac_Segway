/**
 * @file orientation.hpp
 * @author Yang-Rui Li (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <iir_filter2.h>
#include <madgwick_ahrs.h>
#include <config.h>

 /**
  * @brief 9-DoF IMU data structure
  *
  */
typedef struct IMU
{
    IMU()
    {
        memset(this, 0, sizeof(IMU));
    }
    double gyro[3];
    double accel[3];
    double mag[3];
} imu_data_t;

/**
 * @brief This class performs
 * (1) IMU calibration
 * (2) IMU data filtering
 * (3) Estimate orientation
 *
 */
class Orientation
{
public:
    Orientation()
    {
        // Set IMU calibration parameters
        scale_mag[0] = IMU_CAL_MAG_SCALE_X;
        scale_mag[1] = IMU_CAL_MAG_SCALE_Y;
        scale_mag[2] = IMU_CAL_MAG_SCALE_Z;
        bias_mag[0] = IMU_CAL_MAG_BIAS_X;
        bias_mag[1] = IMU_CAL_MAG_BIAS_Y;
        bias_mag[2] = IMU_CAL_MAG_BIAS_Z;
        scale_accel[0] = IMU_CAL_ACCEL_SCALE_X;
        scale_accel[1] = IMU_CAL_ACCEL_SCALE_Y;
        scale_accel[2] = IMU_CAL_ACCEL_SCALE_Z;
        bias_accel[0] = IMU_CAL_ACCEL_BIAS_X;
        bias_accel[1] = IMU_CAL_ACCEL_BIAS_Y;
        bias_accel[2] = IMU_CAL_ACCEL_BIAS_Z;
        memset(gyro_bias, 0, sizeof(gyro_bias));
    }
    void init(double T, const double* accel, const double* mag, const double* _gyro_bias)
    {
        // Set IMU filter parameters
        double fc_accel = 100.0;
        double fc_gyro = 30.0;
        double fc_mag = 100.0;

        // Initialize IMU LPFs
        for (int i = 0; i < 3; i++)
        {
            accel_lpf[i].init_LPF(fc_accel, fc_accel, T);
            gyro_lpf[i].init_LPF(fc_gyro, fc_gyro, T);
            mag_lpf[i].init_LPF(fc_mag, fc_mag, T);
        }

        // Set gyro bias 
        set_gyro_bias(_gyro_bias);

        // Calibration accel and mag data
        for (int i = 0; i < 3; i++)
        {
            imu_cal.accel[i] = scale_accel[i] * (accel[i] + bias_accel[i]);
            imu_cal.mag[i] = scale_mag[i] * (mag[i] + bias_mag[i]);
        }

        // Initialize AHRS
        float beta = 0.1;
        ahrs_magwick.set_filter_params(beta, T);
        ahrs_magwick.set_init_orientation(imu_cal.accel, imu_cal.mag);
    }
    void set_gyro_bias(const double* _gyro_bias)
    {
        memcpy(gyro_bias, _gyro_bias, sizeof(gyro_bias));
    }

    void update(const imu_data_t& imu_raw)
    {
        // IMU calibration
        for (int i = 0; i < 3; i++)
        {
            imu_cal.accel[i] = scale_accel[i] * (imu_raw.accel[i] + bias_accel[i]);
            imu_cal.mag[i] = scale_mag[i] * (imu_raw.mag[i] + bias_mag[i]);
            imu_cal.gyro[i] = imu_raw.gyro[i] - gyro_bias[i];
        }

        // IMU filtering
        for (int i = 0; i < 3; i++)
        {
            imu_filtered.accel[i] = accel_lpf[i].update(imu_cal.accel[i]);
            imu_filtered.gyro[i] = gyro_lpf[i].update(imu_cal.gyro[i]);
            imu_filtered.mag[i] = mag_lpf[i].update(imu_cal.mag[i]);
        }

        // Estimate Orientation
        ahrs_magwick.update(imu_filtered.gyro[0], imu_filtered.gyro[1], imu_filtered.gyro[2],
            imu_filtered.accel[0], imu_filtered.accel[1], imu_filtered.accel[2],
            imu_filtered.mag[0], imu_filtered.mag[1], imu_filtered.mag[2]);

        // Reference Euler angle
        accel_to_roll_pitch(imu_filtered.accel, euler_ref[0], euler_ref[1]);
        mag_to_yaw(imu_filtered.mag, euler_ref[0], euler_ref[1], euler_ref[2]);
    }
    const double* get_cal_accel()
    {
        return imu_cal.accel;
    }
    const double* get_cal_mag()
    {
        return imu_cal.mag;
    }
    const double* get_cal_gyro()
    {
        return imu_cal.gyro;
    }
    const double* get_filtered_gyro()
    {
        return imu_filtered.gyro;
    }
    const double* get_filtered_accel()
    {
        return imu_filtered.accel;
    }
    const double* get_quaternion()
    {
        return ahrs_magwick.get_quat();
    }
    const double* get_euler()
    {
        return ahrs_magwick.get_euler();
    }
    const double* get_ref_euler()
    {
        return euler_ref;
    }
private:
    imu_data_t imu_cal, imu_filtered;
    IIRFilter2 accel_lpf[3];
    IIRFilter2 gyro_lpf[3];
    IIRFilter2 mag_lpf[3];
    double scale_accel[3];
    double bias_accel[3];
    double scale_mag[3];
    double bias_mag[3];
    double gyro_bias[3];
    MadgwickAHRS ahrs_magwick;
    double euler_ref[3];
    // Friends
    friend void print_data();
};
