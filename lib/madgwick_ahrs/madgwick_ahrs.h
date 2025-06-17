// 2022
#pragma once

#include <math.h>
#include <geometry_tool.h>

class MadgwickAHRS
{

public:
    MadgwickAHRS(void)
    {
        // Default filter parameters
        beta = 0.5;
        T = (float)(1.0 / 400.0);

        // Default initial condition
        q0 = 1.0;
        q1 = 0.0;
        q2 = 0.0;
        q3 = 0.0;
    }

    void set_filter_params(float _beta, float _T);
    void set_init_orientation(const double* _accel, const double* _mag);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    inline void get_quat(double* _quat) {
        memcpy(_quat, quat, sizeof(double) * 4U);
    }
    inline void get_euler(double* _euler) {
        memcpy(_euler, euler, sizeof(double) * 3U);
    }
    inline const double* get_quat(void) { return quat; }
    inline const double* get_euler(void) { return euler; }

private:
    static float inv_sqrt(float x);
    float beta; // algorithm gain
    float q0;
    float q1;
    float q2;
    float q3; // quaternion of sensor frame relative to auxiliary frame
    float T;
    double quat[4];
    double euler[3];

};
