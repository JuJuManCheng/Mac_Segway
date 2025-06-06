import numpy as np
import glob
from scipy.optimize import least_squares

def ellipsoid_residuals(params, x):
    offset = params[:3]
    scale = params[3:]
    x_corr =  scale * (x - offset) 
    return np.sum(x_corr**2, axis=1) - 9.81**2

def calibrate_ellipsoid(data):
    offset0 = np.mean(data, axis=0)
    scale0 = np.std(data, axis=0)
    x0 = np.hstack([offset0, scale0])
    res = least_squares(ellipsoid_residuals, x0, args=(data,), method='lm')
    return res.x

# Gather all acc_raw*.csv files
files = glob.glob('imu_raw_data/acc_raw*.csv')
print(f"Found files: {files}")

all_acc = []
for fname in files:
    # Skip the first row (not a header, just numbers)
    data = np.loadtxt(fname, delimiter=',', skiprows=1, usecols=(0,1,2))
    all_acc.append(data)
accel = np.vstack(all_acc)
print(f"Total samples: {accel.shape[0]}")

# Calibrate
params = calibrate_ellipsoid(accel)
print("Accelerometer calibration results:")
print("  Offset:", params[:3])
print("  Scale: ", params[3:])

# Compute and print the norm of the error (residuals)
residuals = ellipsoid_residuals(params, accel)
error_norm = np.linalg.norm(residuals)
print(f"Norm of calibration error: {error_norm:.6f}")

# Save calibration
np.savez('acc_calibration.npz', offset=params[:3], scale=params[3:])

