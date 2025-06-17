# Madgwick AHRS

- This is a C++ wrapper library based on the Madgwick AHRS algorithm.
- Algorithms
  - Can update using only accelerometer + gyroscope data.
  - Can also update with additional magnetometer data.
- Data requirements:
  - Calibrated IMU information is needed. The user does not need to normalize the units, as this will be done internally in the library.
  - Utilizes the definition of the z-axis pointing upwards, with the x-axis oriented towards magnetic north.
