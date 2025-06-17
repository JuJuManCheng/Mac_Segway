/**
 * @file geometry_tool.h
 * @author Yang-Rui Li (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Arduino.h>

/**
 * @brief Determine roll and pitch angles by the accelerometer measurements.
 * 
 * @param accelerometer Accelerometer measurements
 * @param roll Roll angle 
 * @param pitch Pitch angle
 */
void accel_to_roll_pitch(const double* accelerometer, double& roll, double& pitch);

/**
 * @brief Determine yaw angle by tilt angle and magnetometer measurement
 * 
 * @param magnetometer Magnetometer measurements
 * @param roll Roll angle
 * @param pitch Pitch angle
 * @param yaw Yaw angle
 */
void mag_to_yaw(const double* magnetometer, const double& roll, const double& pitch, double& yaw);


/**
 * @brief Determine quaternion (qw,qx,qy,qz) by the Euler angle (roll, pitch, yaw) with 3-2-1 convention
 * 
 * @param euler321 Euler angle (3-2-1)
 * @param quat Quaternion
 */
void euler_to_quat(const double* euler321, double* quat);

/**
 * @brief Determine Euler angle (roll,pitch,yaw) with 3-2-1 convention by the quaternion
 * 
 * @param quat Quaternion
 * @param euler321 Euler angle (3-2-1)
 */
void quat_to_euler(const double* quat, double* euler321);
