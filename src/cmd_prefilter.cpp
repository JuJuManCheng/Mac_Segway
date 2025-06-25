#include "cmd_prefilter.hpp"

cmd_prefilter::cmd_prefilter() {
    x.fill(0.0);
    y.fill(0.0);
}

void cmd_prefilter::pre_filter_int(double fc, double _dt) {
    dt = _dt;
    tao = 1.0 / (2.0 * M_PI * fc);
    A = {0.0, 1.0, 0.0};
    C = {1.0 / pow(tao, 3), 1.0 / pow(tao, 3), 1.0 / pow(tao, 3)};
    B = {0.0, 0.0, 1.0};
    x.fill(0.0);
    y.fill(0.0);
}

void cmd_prefilter::pre_filter_pos_int() {
    x.fill(0.0);
    y.fill(0.0);
}

void cmd_prefilter::pre_filter_bilinear() {
    // For simplicity, this is a placeholder. You may want to use a proper matrix library for real use.
    // Here, we just set Ad and Bd to identity and B for demonstration.
    Ad = {1.0, 0.0, 0.0};
    Bd = {0.0, 0.0, dt};
}

void cmd_prefilter::pre_filter_bilinear_get(double cmd) {
    // Placeholder for bilinear transform update. In real use, replace with matrix operations.
    for (int i = 0; i < 3; ++i) {
        x[i] = Ad[i] * x[i] + Bd[i] * (cmd + cmd_p);
        y[i] = C[i] * x[i];
    }
    cmd_p = cmd;
} 