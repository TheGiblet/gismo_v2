import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Create a PCA9685 object.
pca = PCA9685(i2c)

# Set the PWM frequency to 60Hz (a common value for servos).
pca.frequency = 60

# Define the PCA9685 channel for the servo (adjust as needed).
SERVO_CHANNEL = 1  # Replace with the actual channel

# Function to set servo pulse width directly.
def set_servo_pulse(channel, pulse):
    """Sets the pulse width of the servo motor directly.

    Args:
      channel: The PCA9685 channel connected to the servo.
      pulse: The desired pulse width.
    """
    pca.channels[channel].duty_cycle = pulse
    print(f"Servo channel {channel} pulse width set to {pulse}")

# Manual pulse width control loop.
while True:
    try:
        pulse = int(input("Enter desired servo pulse width: "))
        set_servo_pulse(SERVO_CHANNEL, pulse)
        time.sleep(1)  # Pause to observe the servo's position

    except ValueError:
        print("Invalid input. Please enter an integer.")