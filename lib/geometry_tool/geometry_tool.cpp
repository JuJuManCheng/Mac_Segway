/**
 * @file geometry_tool.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "geometry_tool.h"

void accel_to_roll_pitch(const double* accelerometer, double& roll, double& pitch)
{
    const double& ax = accelerometer[0];
    const double& ay = accelerometer[1];
    const double& az = accelerometer[2];

    double accel_norm = sqrt(ax * ax + ay * ay + az * az);
    /* Determine Roll and Pitch Angles based on Gravity Measurements */
    roll = atan2(ay, az);
    pitch = -asin(ax / accel_norm);
}

void mag_to_yaw(const double* magnetometer, const double& roll, const double& pitch, double& yaw)
{
    const double& mx = magnetometer[0];
    const double& my = magnetometer[1];
    const double& mz = magnetometer[2];

    /* Auxiliary variables to avoid repeated calculation */
    double sphi = sin(roll);
    double cphi = cos(roll);
    double stheta = sin(pitch);
    double ctheta = cos(pitch);

    /* Determine Yaw Angle based on Previous Step Fused Roll and Pitch Angles and Current Step Compass Measurement */
    yaw = -atan2((my * cphi - mz * sphi), (mx * ctheta + my * sphi * stheta + mz * cphi * stheta));
}

void euler_to_quat(const double* euler321, double* quat)
{
    const double& phi = euler321[0];
    const double& theta = euler321[1];
    const double& psi = euler321[2];

    double cphi = std::cos(phi / 2.0);
    double ctheta = std::cos(theta / 2.0);
    double cpsi = std::cos(psi / 2.0);
    double sphi = std::sin(phi / 2.0);
    double stheta = std::sin(theta / 2.0);
    double spsi = std::sin(psi / 2.0);

    quat[0] = cphi * ctheta * cpsi + sphi * stheta * spsi;
    quat[1] = sphi * ctheta * cpsi - cphi * stheta * spsi;
    quat[2] = cphi * stheta * cpsi + sphi * ctheta * spsi;
    quat[3] = cphi * ctheta * spsi - sphi * stheta * cpsi;

    double quat_norm = sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);

    quat[0] /= quat_norm;
    quat[1] /= quat_norm;
    quat[2] /= quat_norm;
    quat[3] /= quat_norm;
}

void quat_to_euler(const double* quat, double* euler321)
{
    const double& q0 = quat[0];
    const double& q1 = quat[1];
    const double& q2 = quat[2];
    const double& q3 = quat[3];

    double q0q1 = q0 * q1;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q1q1 = q1 * q1;
    double q0q2 = q0 * q2;
    double q1q3 = q1 * q3;
    double q0q3 = q0 * q3;
    double q1q2 = q1 * q2;
    double q3q3 = q3 * q3;

    euler321[0] = atan2(2.0 * (q0q1 + q2q3), 1.0 - 2.0 * (q1q1 + q2q2));
    euler321[1] = asin(2.0 * (q0q2 - q1q3));
    euler321[2] = atan2(2.0 * (q0q3 + q1q2), 1.0 - 2.0 * (q2q2 + q3q3));
}
