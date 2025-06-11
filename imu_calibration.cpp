#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;

// Constants
const double GRAVITY = 9.81;
const double LAMBDA_INIT = 0.001;
const double LAMBDA_FACTOR = 10.0;
const int MAX_ITERATIONS = 100;
const double TOLERANCE = 1e-6;

// Structure to hold calibration parameters
struct CalibrationParams {
    Vector3d offset;
    Vector3d scale;
};

// Function to compute residuals
VectorXd computeResiduals(const CalibrationParams& params, const MatrixXd& data) {
    int n = data.rows();
    VectorXd residuals(n);
    
    for (int i = 0; i < n; i++) {
        Vector3d row = data.row(i);
        Vector3d corrected = params.scale.cwiseProduct(row - params.offset);
        residuals(i) = corrected.squaredNorm() - GRAVITY * GRAVITY;
    }
    
    return residuals;
}

// Function to compute Jacobian
MatrixXd computeJacobian(const CalibrationParams& params, const MatrixXd& data) {
    int n = data.rows();
    MatrixXd jacobian(n, 6);
    
    for (int i = 0; i < n; i++) {
        Vector3d row = data.row(i);
        Vector3d corrected = params.scale.cwiseProduct(row - params.offset);
        
        // Derivatives with respect to offset
        jacobian.block<1,3>(i,0) = -2.0 * corrected.transpose().cwiseProduct(params.scale.transpose());
        
        // Derivatives with respect to scale
        jacobian.block<1,3>(i,3) = 2.0 * corrected.transpose().cwiseProduct((row - params.offset).transpose());
    }
    
    return jacobian;
}

// Levenberg-Marquardt algorithm implementation
CalibrationParams levenbergMarquardt(const MatrixXd& data) {
    CalibrationParams params;
    
    // Initialize parameters
    params.offset = data.colwise().mean();
    params.scale = Vector3d::Ones();
    
    double lambda = LAMBDA_INIT;
    double prev_cost = INFINITY;
    
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        // Compute residuals and Jacobian
        VectorXd residuals = computeResiduals(params, data);
        MatrixXd jacobian = computeJacobian(params, data);
        
        // Compute current cost
        double current_cost = residuals.squaredNorm();
        
        // Check convergence
        if (abs(current_cost - prev_cost) < TOLERANCE) {
            break;
        }
        
        // Compute step
        MatrixXd JTJ = jacobian.transpose() * jacobian;
        VectorXd JTr = jacobian.transpose() * residuals;
        
        // Add damping term
        MatrixXd damping = lambda * MatrixXd::Identity(6, 6);
        VectorXd step = (JTJ + damping).ldlt().solve(-JTr);
        
        // Try new parameters
        CalibrationParams new_params = params;
        new_params.offset += step.segment<3>(0);
        new_params.scale += step.segment<3>(3);
        
        // Compute new cost
        VectorXd new_residuals = computeResiduals(new_params, data);
        double new_cost = new_residuals.squaredNorm();
        
        // Update parameters if cost decreased
        if (new_cost < current_cost) {
            params = new_params;
            lambda /= LAMBDA_FACTOR;
            prev_cost = new_cost;
        } else {
            lambda *= LAMBDA_FACTOR;
        }
    }
    
    return params;
}

int main() {
    // Read data from CSV files
    vector<MatrixXd> all_acc;
    string data_dir = "imu_raw_data";
    
    for (const auto& entry : filesystem::directory_iterator(data_dir)) {
        if (entry.path().string().find("acc_raw") != string::npos) {
            ifstream file(entry.path());
            string line;
            vector<vector<double>> data;
            
            // Skip first line
            getline(file, line);
            
            while (getline(file, line)) {
                stringstream ss(line);
                string value;
                vector<double> row;
                
                for (int i = 0; i < 3; i++) {
                    getline(ss, value, ',');
                    row.push_back(stod(value));
                }
                
                data.push_back(row);
            }
            
            MatrixXd matrix(data.size(), 3);
            for (size_t i = 0; i < data.size(); i++) {
                matrix.row(i) = Map<Vector3d>(data[i].data());
            }
            
            all_acc.push_back(matrix);
        }
    }
    
    // Combine all data
    int total_rows = 0;
    for (const auto& acc : all_acc) {
        total_rows += acc.rows();
    }
    
    MatrixXd accel(total_rows, 3);
    int current_row = 0;
    for (const auto& acc : all_acc) {
        accel.block(current_row, 0, acc.rows(), 3) = acc;
        current_row += acc.rows();
    }
    
    cout << "Total samples: " << accel.rows() << endl;
    
    // Calibrate
    CalibrationParams params = levenbergMarquardt(accel);
    
    cout << "Accelerometer calibration results:" << endl;
    cout << "  Offset: " << params.offset.transpose() << endl;
    cout << "  Scale: " << params.scale.transpose() << endl;
    
    // Compute and print the norm of the error
    VectorXd residuals = computeResiduals(params, accel);
    double error_norm = residuals.norm();
    cout << "Norm of calibration error: " << error_norm << endl;
    
    // Save calibration parameters
    ofstream outfile("acc_calibration.txt");
    outfile << "Offset: " << params.offset.transpose() << endl;
    outfile << "Scale: " << params.scale.transpose() << endl;
    outfile.close();
    
    return 0;
} 