#ifndef FILTER_HPP
#define FILTER_HPP

#include <array>
#include <cmath>

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
            filtered_value_ = alpha_ * new_value + (1.0 - alpha_) * filtered_value_;
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

    // Simple Kalman Filter for 1D state
    class SimpleKalmanFilter {
    public:
        SimpleKalmanFilter(double process_noise, double measurement_noise) 
            : process_noise_(process_noise), measurement_noise_(measurement_noise),
              estimate_(0.0), estimate_error_(1.0) {}
        
        double update(double measurement) {
            // Predict
            double prediction_error = estimate_error_ + process_noise_;
            
            // Update
            double kalman_gain = prediction_error / (prediction_error + measurement_noise_);
            estimate_ = estimate_ + kalman_gain * (measurement - estimate_);
            estimate_error_ = (1.0 - kalman_gain) * prediction_error;
            
            return estimate_;
        }

        void reset(double initial_value = 0.0) {
            estimate_ = initial_value;
            estimate_error_ = 1.0;
        }

    private:
        double process_noise_;      // Process noise variance
        double measurement_noise_;  // Measurement noise variance
        double estimate_;          // Current state estimate
        double estimate_error_;    // Current estimate error
    };
};

#endif // FILTER_HPP 