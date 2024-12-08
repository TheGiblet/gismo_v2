# grove_ultrasonic_test_v1.2.py
# Controls two DC motors and a neck servo connected to a PCA9685 PWM driver.
# The robot uses a Grove Ultrasonic Distance Sensor, rotated by the servo, 
# to find a clear path and avoid objects.

import time
from board import SCL, SDA  # Import I2C pins for communication
import busio
import RPi.GPIO as GPIO

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# --- I2C Setup for PCA9685 ---
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# --- GPIO Setup for Ultrasonic Sensor ---
grove_ultrasonic_pin = 25
GPIO.setmode(GPIO.BCM)

# --- Motor and Servo Channels ---
motor1_in1 = 0
motor1_in2 = 1
motor2_in1 = 2
motor2_in2 = 3
servo_channel = 4

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

def set_servo_angle(channel, angle):
    """Sets the angle of the servo motor.

    Args:
        channel: The PCA9685 channel the servo is connected to.
        angle: The desired angle in degrees (-60 to +60).
    """
    if angle < -60:
        angle = -60
    elif angle > 60:
        angle = 60
    pulse = int((angle + 60) / 120 * (65535 - 1500) + 1500)
    pca.channels[channel].duty_cycle = pulse

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
        # Scan for a clear path
        clear_path = False
        for angle in range(-60, 61, 20):  # Scan in 20-degree steps
            set_servo_angle(servo_channel, angle)
            time.sleep(0.2)  # Wait for servo to stabilize
            distance = read_grove_distance(grove_ultrasonic_pin)
            print(f"Angle: {angle}, Distance: {distance:.2f} cm")
            if distance > 30:  # Adjust threshold as needed
                clear_path = True
                break  # Found a clear path, exit the loop

        if clear_path:
            # Move in the direction of the clear path
            if angle < 0:
                # Turn left
                set_motor_speed(motor1_in1, motor1_in2, 50)  # Adjust speed as needed
                set_motor_speed(motor2_in1, motor2_in2, -50)
                print("Turning left")
            elif angle > 0:
                # Turn right
                set_motor_speed(motor1_in1, motor1_in2, -50)  # Adjust speed as needed
                set_motor_speed(motor2_in1, motor2_in2, 50)
                print("Turning right")
            else:
                # Move forward
                set_motor_speed(motor1_in1, motor1_in2, 75)  # Adjust speed as needed
                set_motor_speed(motor2_in1, motor2_in2, 75)
                print("Moving forward")
            time.sleep(1)  # Adjust movement duration as needed
        else:
            # No clear path, stop or take other action
            set_motor_speed(motor1_in1, motor1_in2, 0)
            set_motor_speed(motor2_in1, motor2_in2, 0)
            print("No clear path, stopping")
            time.sleep(1)

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    pca.deinit()
    GPIO.cleanup()
    print("Program stopped.")