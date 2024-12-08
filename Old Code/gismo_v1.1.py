# grove_ultrasonic_test_v1.1.py
# Controls two DC motors connected to a L298N motor driver via a PCA9685 PWM driver.
# The robot uses a Grove Ultrasonic Distance Sensor to avoid objects.

import time
from board import SCL, SDA  # Import I2C pins for communication
import busio
import RPi.GPIO as GPIO

# Import the PCA9685 module for controlling the PWM driver.
from adafruit_pca9685 import PCA9685

# --- I2C Setup for PCA9685 ---
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# --- GPIO Setup for Ultrasonic Sensor ---
grove_ultrasonic_pin = 25
GPIO.setmode(GPIO.BCM)

# --- Motor Channels ---
motor1_in1 = 0
motor1_in2 = 1
motor2_in1 = 2
motor2_in2 = 3

# --- Functions ---

def set_motor_speed(motor_in1, motor_in2, speed):
    """Sets the speed of a motor.

    Args:
        motor_in1: The PCA9685 channel for the first motor input.
        motor_in2: The PCA9685 channel for the second motor input.
        speed: The speed value from -100 (full reverse) to 100 (full forward).
    """
    if speed > 100:
        speed = 100
    if speed < -100:
        speed = -100

    if speed >= 0:
        pca.channels[motor_in1].duty_cycle = int(speed / 100 * 65535)
        pca.channels[motor_in2].duty_cycle = 0
    else:
        pca.channels[motor_in1].duty_cycle = 0
        pca.channels[motor_in2].duty_cycle = int(-speed / 100 * 65535)

def read_grove_distance(pin):
    """Reads the distance from the Grove Ultrasonic Sensor.

    Args:
        pin: The GPIO pin connected to the sensor's signal pin.

    Returns:
        The distance in centimeters.
    """
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(pin, GPIO.LOW)
    start = time.time()

    GPIO.setup(pin, GPIO.IN)
    while GPIO.input(pin) == 0:
        start = time.time()

    while GPIO.input(pin) == 1:
        stop = time.time()

    elapsed = stop - start
    distance = (elapsed * 34300) / 2  # Speed of sound in cm/s

    return distance

# --- Main Program ---
try:
    while True:
        distance = read_grove_distance(grove_ultrasonic_pin)
        print(f"Distance: {distance:.2f} cm")

        if distance < 20:  # Adjust threshold as needed
            # Object detected! Stop and maybe move backward
            set_motor_speed(motor1_in1, motor1_in2, 0)
            set_motor_speed(motor2_in1, motor2_in2, 0)
            time.sleep(0.5)
            set_motor_speed(motor1_in1, motor1_in2, -50)  # Move backward
            set_motor_speed(motor2_in1, motor2_in2, -50)
            time.sleep(1)
            # Add code to turn or change direction here
        else:
            # No object detected, move forward
            set_motor_speed(motor1_in1, motor1_in2, 75)
            set_motor_speed(motor2_in1, motor2_in2, 75)

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    pca.deinit()
    GPIO.cleanup()
    print("Program stopped.")