# gismo_v2.2.py
# This code controls the motors, reads edge sensors, and sets the RGB LED color accordingly.

import time
import board
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO

# Initialize I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Create a PCA9685 object.
pca = PCA9685(i2c)

# Set the PWM frequency to 60Hz.
pca.frequency = 60

# Define the PCA9685 channels connected to the L298N motor driver.
INT1 = 7  
INT2 = 8
INT3 = 10
INT4 = 9

# Define GPIO pins for line tracking sensors
left_sensor_pin = 12
right_sensor_pin = 13

# Define RGB LED channels on PCA9685
red_channel = 13
green_channel = 12
blue_channel = 11

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_sensor_pin, GPIO.IN) 
GPIO.setup(right_sensor_pin, GPIO.IN)

def set_motor_speed(motor, speed):
  """Sets the speed of a motor.

  Args:
    motor: 1 for left motor, 2 for right motor.
    speed: -1 to 1, where -1 is full reverse, 1 is full forward.
  """
  if motor == 1:
    if speed > 0:  # Forward
      pca.channels[INT1].duty_cycle = int(speed * 65535)
      pca.channels[INT2].duty_cycle = 0
    elif speed < 0:  # Reverse
      pca.channels[INT1].duty_cycle = 0
      pca.channels[INT2].duty_cycle = int(-speed * 65535)
    else:  # Stop
      pca.channels[INT1].duty_cycle = 0
      pca.channels[INT2].duty_cycle = 0
  elif motor == 2:
    if speed > 0:  # Forward
      pca.channels[INT3].duty_cycle = int(speed * 65535)
      pca.channels[INT4].duty_cycle = 0
    elif speed < 0:  # Reverse
      pca.channels[INT3].duty_cycle = 0
      pca.channels[INT4].duty_cycle = int(-speed * 65535)
    else:  # Stop
      pca.channels[INT3].duty_cycle = 0
      pca.channels[INT4].duty_cycle = 0

def set_rgb_led(red, green, blue):
    """Sets the color of the RGB LED."""
    pca.channels[red_channel].duty_cycle = int(red * 65535)
    pca.channels[green_channel].duty_cycle = int(green * 65535)
    pca.channels[blue_channel].duty_cycle = int(blue * 65535)

# Main program loop
try:
    while True:
        left_sensor_value = GPIO.input(left_sensor_pin)
        right_sensor_value = GPIO.input(right_sensor_pin)

        if left_sensor_value == 0 or right_sensor_value == 0:  # Edge detected
            print("Edge detected! Stopping.")
            set_motor_speed(1, 0)  # Stop left motor
            set_motor_speed(2, 0)  # Stop right motor
            set_rgb_led(1, 0, 0)  # Red for edge detected
        else:
            # No edge detected, move forward
            set_motor_speed(1, 0.5)  # Left motor forward
            set_motor_speed(2, 0.5)  # Right motor forward
            set_rgb_led(0, 1, 0)  # Green for all clear

        time.sleep(0.1)  # Adjust delay as needed

except KeyboardInterrupt:
    # Stop motors on Ctrl+C
    set_motor_speed(1, 0)
    set_motor_speed(2, 0)
    set_rgb_led(0, 0, 0)  # Turn off the LED
    pca.deinit()
    GPIO.cleanup()  # Clean up GPIO pins