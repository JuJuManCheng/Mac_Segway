#ifndef FILTER_HPP
#define FILTER_HPP

#include <array>
#include <cmath>
#include <Eigen/Dense>

class Filter {
public:
    // Constructor
    Filter() = default;
    virtual ~Filter() = default;

    // First Order Low Pass Filter
    class LowPassFilter1st {
    public:
        LowPassFilter1st(double alpha) : alpha_(alpha), filtered_value_(0.0) {}
        
        double update(double new_value) {
            filtered_value_ = alpha_ * filtered_value_ + (1.0 - alpha_) * new_value;
            return filtered_value_;
        }

        void reset(double initial_value = 0.0) {
            filtered_value_ = initial_value;
        }

    private:
        double alpha_;           // Filter coefficient (0 < alpha < 1)
        double filtered_value_;  // Current filtered value
    };

    // Second Order Low Pass Filter
    class LowPassFilter2nd {
    public:
        LowPassFilter2nd(double alpha1, double alpha2) 
            : alpha1_(alpha1), alpha2_(alpha2), 
              filtered_value1_(0.0), filtered_value2_(0.0) {}
        
        double update(double new_value) {
            filtered_value1_ = alpha1_ * filtered_value1_ + (1.0 - alpha1_) * new_value;
            filtered_value2_ = alpha2_ * filtered_value2_ + (1.0 - alpha2_) * filtered_value1_;
            return filtered_value2_;
        }

        void reset(double initial_value = 0.0) {
            filtered_value1_ = initial_value;
            filtered_value2_ = initial_value;
        }

    private:
        double alpha1_, alpha2_;     // Filter coefficients
        double filtered_value1_;     // First stage filtered value
        double filtered_value2_;     // Second stage filtered value
    };

    // Kalman Filter
    class KalmanFilter {
    public:
        KalmanFilter(int state_dim, int measurement_dim) 
            : state_dim_(state_dim), measurement_dim_(measurement_dim) {
            // Initialize matrices
            x_ = Eigen::VectorXd::Zero(state_dim_);        // State estimate
            P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);  // Error covariance
            Q_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);  // Process noise
            R_ = Eigen::MatrixXd::Identity(measurement_dim_, measurement_dim_);  // Measurement noise
            H_ = Eigen::MatrixXd::Zero(measurement_dim_, state_dim_);  // Measurement matrix
            F_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);  // State transition matrix
        }

        void setProcessNoise(const Eigen::MatrixXd& Q) { Q_ = Q; }
        void setMeasurementNoise(const Eigen::MatrixXd& R) { R_ = R; }
        void setMeasurementMatrix(const Eigen::MatrixXd& H) { H_ = H; }
        void setStateTransitionMatrix(const Eigen::MatrixXd& F) { F_ = F; }

        Eigen::VectorXd update(const Eigen::VectorXd& measurement) {
            // Predict
            x_ = F_ * x_;
            P_ = F_ * P_ * F_.transpose() + Q_;

            // Update
            Eigen::MatrixXd K = P_ * H_.transpose() * 
                               (H_ * P_ * H_.transpose() + R_).inverse();
            x_ = x_ + K * (measurement - H_ * x_);
            P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H_) * P_;

            return x_;
        }

        void reset(const Eigen::VectorXd& initial_state) {
            x_ = initial_state;
            P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        }

    private:
        int state_dim_;
        int measurement_dim_;
        Eigen::VectorXd x_;      // State estimate
        Eigen::MatrixXd P_;      // Error covariance
        Eigen::MatrixXd Q_;      // Process noise
        Eigen::MatrixXd R_;      // Measurement noise
        Eigen::MatrixXd H_;      // Measurement matrix
        Eigen::MatrixXd F_;      // State transition matrix
    };

    // Extended Kalman Filter
    class ExtendedKalmanFilter {
    public:
        ExtendedKalmanFilter(int state_dim, int measurement_dim) 
            : state_dim_(state_dim), measurement_dim_(measurement_dim) {
            // Initialize matrices
            x_ = Eigen::VectorXd::Zero(state_dim_);        // State estimate
            P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);  // Error covariance
            Q_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);  // Process noise
            R_ = Eigen::MatrixXd::Identity(measurement_dim_, measurement_dim_);  // Measurement noise
        }

        void setProcessNoise(const Eigen::MatrixXd& Q) { Q_ = Q; }
        void setMeasurementNoise(const Eigen::MatrixXd& R) { R_ = R; }

        // Function pointers for state transition and measurement functions
        using StateTransitionFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
        using MeasurementFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
        using JacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;

        void setStateTransitionFunction(StateTransitionFunc f) { f_ = f; }
        void setMeasurementFunction(MeasurementFunc h) { h_ = h; }
        void setStateTransitionJacobian(JacobianFunc F) { F_ = F; }
        void setMeasurementJacobian(JacobianFunc H) { H_ = H; }

        Eigen::VectorXd update(const Eigen::VectorXd& measurement) {
            // Predict
            x_ = f_(x_);
            Eigen::MatrixXd F = F_(x_);
            P_ = F * P_ * F.transpose() + Q_;

            // Update
            Eigen::MatrixXd H = H_(x_);
            Eigen::MatrixXd K = P_ * H.transpose() * 
                               (H * P_ * H.transpose() + R_).inverse();
            x_ = x_ + K * (measurement - h_(x_));
            P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_;

            return x_;
        }

        void reset(const Eigen::VectorXd& initial_state) {
            x_ = initial_state;
            P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        }

    private:
        int state_dim_;
        int measurement_dim_;
        Eigen::VectorXd x_;      // State estimate
        Eigen::MatrixXd P_;      // Error covariance
        Eigen::MatrixXd Q_;      // Process noise
        Eigen::MatrixXd R_;      // Measurement noise
        
        // Function pointers
        StateTransitionFunc f_;  // State transition function
        MeasurementFunc h_;      // Measurement function
        JacobianFunc F_;         // State transition Jacobian
        JacobianFunc H_;         // Measurement Jacobian
    };
};

#endif // FILTER_HPP 