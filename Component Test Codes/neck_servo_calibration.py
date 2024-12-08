# neck_servo_calibration.py
# Calibrates the neck servo by rotating it to its minimum and maximum angles,
# then finds the center position and provides distance readings at each location.

import time
from board import SCL, SDA  # Import I2C pins for communication
import busio

# Import the PCA9685 module for controlling the PWM driver.
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

# Create a PCA9685 class instance.
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz, suitable for most servos.
pca.frequency = 60

# Define the servo channel
servo_channel = 4

# Function to set servo angle
def set_servo_angle(channel, angle):
    """Sets the angle of a servo motor.

    Args:
        channel: The PCA9685 channel the servo is connected to.
        angle: The desired angle in degrees (0 to 180).
    """
    if angle < 0:
        angle = 0  # Ensure angle is not less than 0
    elif angle > 180:
        angle = 180  # Ensure angle is not greater than 180

    # Calculate the pulse width for the servo. 1500 is typically the center position.
    pulse = int(angle / 180 * (65535 - 1500) + 1500)
    pca.channels[channel].duty_cycle = pulse  # Set the duty cycle for the channel

# Function to read distance from the ultrasonic sensor (replace with your actual implementation)
def read_distance():
    """Reads the distance from the ultrasonic sensor.

    Returns:
        The distance in centimeters.
    """
    # Replace this with your actual code to read from the ultrasonic sensor
    # This is just a placeholder
    # You'll need to use the `RPi.GPIO` library and the appropriate trigger/echo pins
    # to interact with the ultrasonic sensor.
    time.sleep(0.1)  # Simulate sensor reading delay
    return 20  # Example distance

try:
    # --- Minimum Angle ---
    print("Moving to minimum angle...")
    set_servo_angle(servo_channel, 0)  # Move to minimum angle (0 degrees)
    time.sleep(2)  # Wait for servo to reach the position
    min_distance = read_distance()
    print(f"Minimum Angle Distance: {min_distance} cm")

    # --- Maximum Angle ---
    print("Moving to maximum angle...")
    set_servo_angle(servo_channel, 180)  # Move to maximum angle (180 degrees)
    time.sleep(2)  # Wait for servo to reach the position
    max_distance = read_distance()
    print(f"Maximum Angle Distance: {max_distance} cm")

    # --- Center Position ---
    print("Moving to center position...")
    set_servo_angle(servo_channel, 90)  # Move to the center position (90 degrees)
    time.sleep(2)  # Wait for servo to reach the position
    center_distance = read_distance()
    print(f"Center Position Distance: {center_distance} cm")

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    pca.deinit()  # Deinitialize the PCA9685
    print("Servo calibration stopped.")