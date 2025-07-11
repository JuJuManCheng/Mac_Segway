class LPF3_StateSpace {
public:
    LPF3_StateSpace(double sample_period, double cut_off_Hz, double damping1 = 1.0, double damping2 = 1.0) {
        T = sample_period;
        omega_n = 2.0 * PI * cut_off_Hz;

        // 預設為連乘阻尼型態 damping1, damping2 用於決定系統特性
        a0 = omega_n * omega_n * omega_n;
        a1 = damping1 * omega_n * omega_n * 3.0;
        a2 = damping2 * omega_n * 3.0;

        x1 = x2 = x3 = 0.0;
    }

    double update(double u) {
        // Euler integration
        double x1_next = x1 + T * x2;
        double x2_next = x2 + T * x3;
        double x3_next = x3 + T * (-a2 * x3 - a1 * x2 - a0 * x1 + a0 * u);

        x1 = x1_next;
        x2 = x2_next;
        x3 = x3_next;

        return x1;
    }

    double x_dot() const {
        return x2;
    }

    double x_ddot() const {
        return x3;
    }

    void reset(double val = 0.0) {
        x1 = val;
        x2 = 0.0;
        x3 = 0.0;
    }

private:
    double T, omega_n;
    double a0, a1, a2;
    double x1, x2, x3;
};
