# motor_control.py
# Controls two DC motors connected to a L298N motor driver 
# via a PCA9685 PWM driver.

import time
from board import SCL, SDA  # Import I2C pins for communication
import busio

# Import the PCA9685 module for controlling the PWM driver.
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

# Create a PCA9685 class instance.
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz, suitable for most motors.
pca.frequency = 60

# Define the motor channels on the PCA9685
motor1_in1 = 0  # Channel 0 connected to IN1 of motor driver 1
motor1_in2 = 1  # Channel 1 connected to IN2 of motor driver 1
motor2_in1 = 2  # Channel 2 connected to IN1 of motor driver 2
motor2_in2 = 3  # Channel 3 connected to IN2 of motor driver 2

# Function to set motor speed
def set_motor_speed(motor_in1, motor_in2, speed):
    """Sets the speed of a motor.

    Args:
        motor_in1: The PCA9685 channel for the first motor input.
        motor_in2: The PCA9685 channel for the second motor input.
        speed: The speed value from -100 (full reverse) to 100 (full forward).
    """
    if speed > 100:
        speed = 100  # Limit speed to 100
    if speed < -100:
        speed = -100  # Limit speed to -100

    if speed >= 0:
        pca.channels[motor_in1].duty_cycle = int(speed / 100 * 65535)  # Set PWM for forward motion
        pca.channels[motor_in2].duty_cycle = 0
    else:
        pca.channels[motor_in1].duty_cycle = 0
        pca.channels[motor_in2].duty_cycle = int(-speed / 100 * 65535)  # Set PWM for reverse motion

try:
  while True:
      # Move motor 1 forward at full speed
      set_motor_speed(motor1_in1, motor1_in2, 100)
      time.sleep(2)  # Wait for 2 seconds

      # Stop motor 1
      set_motor_speed(motor1_in1, motor1_in2, 0)
      time.sleep(1)  # Wait for 1 second

      # Move motor 1 backward at half speed
      set_motor_speed(motor1_in1, motor1_in2, -50)
      time.sleep(2)  # Wait for 2 seconds

      # Stop motor 1
      set_motor_speed(motor1_in1, motor1_in2, 0)
      time.sleep(1)  # Wait for 1 second

      # Move motor 2 forward at 75% speed
      set_motor_speed(motor2_in1, motor2_in2, 75)
      time.sleep(2)  # Wait for 2 seconds

      # Stop motor 2
      set_motor_speed(motor2_in1, motor2_in2, 0)
      time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
  # Clean up on exit (Ctrl+C)
  pca.deinit()  # Deinitialize the PCA9685
  print("Motor control stopped.")