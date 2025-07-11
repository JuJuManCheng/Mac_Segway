import numpy as np
import glob
from scipy.optimize import least_squares

def ellipsoid_residuals(params, x):
    offset = params[:3]
    scale = params[3:]
    x_corr =  scale * (x - offset) 
    return np.sum(x_corr**2, axis=1) - 32.0**2

def calibrate_ellipsoid(data):
    offset0 = np.mean(data, axis=0)
    scale0 = np.std(data, axis=0)
    x0 = np.hstack([offset0, scale0])
    res = least_squares(ellipsoid_residuals, x0, args=(data,), method='lm')
    return res.x

# Gather all acc_raw*.csv files
# files = glob.glob('imu_raw_data/acc_raw*.csv')
files = glob.glob('imu_raw_data/mag_raw*.csv')
print(f"Found files: {files}")

all_acc = []
all_mag = []
all_gyro = []
for fname in files:
    # Skip the first row (not a header, just numbers)
    acc_data = np.loadtxt(fname, delimiter=',', skiprows=1, usecols=(0,1,2))
    gyro_data = np.loadtxt(fname, delimiter=',', skiprows=1, usecols=(3,4,5))
    mag_data = np.loadtxt(fname, delimiter=',', skiprows=1, usecols=(6,7,8))

    all_acc.append(acc_data)
    all_mag.append(mag_data)
    all_gyro.append(gyro_data)



acc = np.vstack(all_acc)
# print(f"Total samples: {acc.shape[0]}")
mag = np.vstack(all_mag)
# print(f"Total samples: {mag.shape[0]}")
gyro = np.vstack(all_gyro)
gyro_offset = np.mean(gyro, axis=0)
# print(f"Total samples: {gyro.shape[0]}")

# Calibrate
# params = calibrate_ellipsoid(acc)
# print(" Accelerometer Calibration results:")
# print("  Offset:", params[:3])
# print("  Scale: ", params[3:])

params = calibrate_ellipsoid(mag)
print(" Magnetometer Calibration results:")
print("  Offset:", params[:3])
print("  Scale: ", params[3:])

# print("gyroscope calibration results:")
# print("  Offset: ", gyro_offset)

# Compute and print the norm of the error (residuals)
# residuals = ellipsoid_residuals(params, mag)
# error_norm = np.linalg.norm(residuals)
# print(f"Norm of calibration error: {error_norm:.6f}")

# # Save calibration
# np.savez('acc_calibration.npz', offset=params[:3], scale=params[3:])

