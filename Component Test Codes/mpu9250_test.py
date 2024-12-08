# mpu9250_test.py
# Tests the MPU9250 9-axis IMU connected to the I2C bus.

import time

from mpu9250_jmdev.mpu_9250 import MPU9250

# Initialize the IMU
imu = MPU9250(address_ak=0x68, bus=1)
imu.configure()

try:
    while True:
        # Read accelerometer, gyroscope, and magnetometer data
        accel = imu.readAccelerometerMaster()  # Corrected function name
        gyro = imu.readGyroscopeMaster()      # Corrected function name
        mag = imu.readMagnetometerMaster()    # Corrected function name

        # Print the data
        print(f"Acceleration: {accel}")
        print(f"Gyroscope: {gyro}")
        print(f"Magnetometer: {mag}")
        print("-" * 20)

        time.sleep(0.1)  # Adjust delay as needed

except KeyboardInterrupt:
    print("MPU9250 test stopped.")