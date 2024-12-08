# servo_calibration.py
# Calibrates the neck, left lifter, and right lifter servos (SG90) by rotating 
# them to their minimum and maximum angles, then finds the center position.
# Provides feedback readings (replace placeholders with your sensor reading code)
# at each location.

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

# Define the servo channels
neck_servo_channel = 4
left_lifter_channel = 5
right_lifter_channel = 6

# --- Servo Calibration Functions ---

def set_servo_pulse(channel, pulse):
    """Sets the servo position by pulse width.

    Args:
        channel: The PCA9685 channel the servo is connected to.
        pulse: The pulse width in microseconds (900-2100 for SG90 servos).
    """
    # Calculate the duty cycle value for the PCA9685
    duty_cycle = int(pulse * 65535 / 20000)  # 20000 microseconds = 100% duty cycle
    pca.channels[channel].duty_cycle = duty_cycle

def get_feedback():
    """Gets feedback from the system about the servo position.

    Returns:
        The feedback value (e.g., distance, angle, encoder reading).
    """
    # TODO: Replace this with your actual code to read from your sensor
    # This is just a placeholder
    time.sleep(0.1)  # Simulate sensor reading delay
    return 20  # Example distance

# --- Calibration Procedure ---
try:
    for channel in [neck_servo_channel, left_lifter_channel, right_lifter_channel]:
        print(f"Calibrating servo on channel {channel}...")

        # 1. Move to minimum position 
        print("  Moving to minimum position...")
        set_servo_pulse(channel, 900)  # Adjusted minimum pulse width for SG90
        time.sleep(2)  # Wait for servo to reach position
        min_feedback = get_feedback()
        print(f"  Minimum Position Feedback: {min_feedback}")

        # 2. Move to maximum position 
        print("  Moving to maximum position...")
        set_servo_pulse(channel, 2100)  # Adjusted maximum pulse width for SG90
        time.sleep(2)
        max_feedback = get_feedback()
        print(f"  Maximum Position Feedback: {max_feedback}")

        # 3. Calculate and move to center position
        center_pulse = (900 + 2100) / 2  # Adjusted range for SG90
        print("  Moving to center position...")
        set_servo_pulse(channel, center_pulse)
        time.sleep(2)
        center_feedback = get_feedback()
        print(f"  Center Position Feedback: {center_feedback}")

        print("-" * 20)  # Separator between servos

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    pca.deinit()  # Deinitialize the PCA9685
    print("Servo calibration stopped.")