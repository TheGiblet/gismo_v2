# gismo_v2.4.py
# This code controls the motors, reads edge sensors, sets the RGB LED,
# makes the robot "waggle" when the touch sensor is activated, and
# shakes arms when an edge is detected.

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

# Define GPIO pin for touch sensor
touch_sensor_pin = 17  # Adjust if needed

# Define servo channels on PCA9685
left_servo_channel = 0
right_servo_channel = 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_sensor_pin, GPIO.IN) 
GPIO.setup(right_sensor_pin, GPIO.IN)
GPIO.setup(touch_sensor_pin, GPIO.IN)

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

def waggle():
    """Makes the robot "waggle" its motors."""
    for _ in range(5):  # Adjust the number of waggles
        set_motor_speed(1, 0.75)  # Left motor forward
        set_motor_speed(2, -0.75) # Right motor backward
        time.sleep(0.2)  # Adjust waggle duration
        set_motor_speed(1, -0.75) # Left motor backward
        set_motor_speed(2, 0.75)  # Right motor forward
        time.sleep(0.2)
    # Stop motors after waggling
    set_motor_speed(1, 0)
    set_motor_speed(2, 0)

def set_servo_angle(channel, angle):
    """Sets the angle of a servo motor."""
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    pulse = int(angle / 180 * (65535 - 1500) + 1500)  # Calculate pulse width
    pca.channels[channel].duty_cycle = pulse

def shake_arms():
    """Makes the robot shake its arms."""
    for _ in range(5):  # Adjust the number of shakes
        # Move arms to one side
        set_servo_angle(left_servo_channel, 180)  # Left arm to the left
        set_servo_angle(right_servo_channel, 180)  # Right arm to the left
        time.sleep(0.2)  # Adjust shake duration

        # Move arms to the other side
        set_servo_angle(left_servo_channel, 0)  # Left arm to the right
        set_servo_angle(right_servo_channel, 0)  # Right arm to the right
        time.sleep(0.2)

    # Return arms to neutral position
    set_servo_angle(left_servo_channel, 90)  # Left arm to the center
    set_servo_angle(right_servo_channel, 90)  # Right arm to the center

# Main program loop
try:
    while True:
        left_sensor_value = GPIO.input(left_sensor_pin)
        right_sensor_value = GPIO.input(right_sensor_pin)

        if left_sensor_value == 1 or right_sensor_value == 1:  # Edge detected (line detected)
            print("Edge detected! Stopping.")
            set_motor_speed(1, 0)  # Stop left motor
            set_motor_speed(2, 0)  # Stop right motor
            set_rgb_led(1, 0, 0)  # Red for edge detected
            shake_arms()  # Shake arms when edge detected
        else:
            # No edge detected, move forward
            set_motor_speed(1, 0.5)  # Left motor forward
            set_motor_speed(2, 0.5)  # Right motor forward
            set_rgb_led(0, 1, 0)  # Green for all clear

        # Check touch sensor
        if GPIO.input(touch_sensor_pin) == GPIO.HIGH:
            print("Touch detected! Waggle time!")
            waggle()
            set_rgb_led(0, 0, 1)  # Blue for touch

        time.sleep(0.1)  # Adjust delay as needed

except KeyboardInterrupt:
    # Stop motors on Ctrl+C
    set_motor_speed(1, 0)
    set_motor_speed(2, 0)
    set_rgb_led(0, 0, 0)  # Turn off the LED
    pca.deinit()
    GPIO.cleanup()  # Clean up GPIO pins