#pragma once
#include <array>
#include <cmath>

class cmd_prefilter {
private:
    std::array<double, 3> A, Ad, C, num;
    std::array<double, 3> B, Bd, x;
    double I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    double tao;
    double cmd_p = 0;
    double dt;
public:
    std::array<double, 3> y;
    cmd_prefilter();
    void pre_filter_int(double fc, double _dt);
    void pre_filter_pos_int();
    void pre_filter_bilinear();
    void pre_filter_bilinear_get(double cmd);
}; 