# gismo_v1.4.1.py
# Combines motor control, neck servo, Grove Ultrasonic sensor, edge detection,
# and buzzer feedback with melodies. The robot prioritizes edge detection, 
# scans for a clear path, avoids objects, backs up and turns when an edge 
# is detected, and uses the buzzer to play different melodies for object 
# detection and edge detection.

import time
from board import SCL, SDA
import busio
import RPi.GPIO as GPIO

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# --- I2C Setup for PCA9685 ---
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# --- GPIO Setup for Ultrasonic Sensor, Edge Sensors, and Buzzer ---
grove_ultrasonic_pin = 25
left_edge_pin = 12
right_edge_pin = 13
buzzer_pin = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_edge_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(right_edge_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buzzer_pin, GPIO.OUT)

# --- Motor and Servo Channels ---
motor1_in1 = 0
motor1_in2 = 1
motor2_in1 = 2
motor2_in2 = 3
servo_channel = 4

# --- Functions ---

def set_motor_speed(motor_in1, motor_in2, speed):
    """Sets the speed of a motor."""
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
    """Sets the angle of the servo motor."""
    if angle < -60:
        angle = -60
    elif angle > 60:
        angle = 60
    pulse = int((angle + 60) / 120 * (65535 - 1500) + 1500)
    pca.channels[channel].duty_cycle = pulse

def read_grove_distance(pin):
    """Reads the distance from the Grove Ultrasonic Sensor."""
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
    distance = (elapsed * 34300) / 2
    return distance

def play_tone(frequency, duration):
    """Plays a tone on the buzzer for a given duration."""
    pwm = GPIO.PWM(buzzer_pin, frequency)
    pwm.start(50)
    time.sleep(duration)
    pwm.stop()

def play_melody(melody):
    """Plays a melody on the buzzer."""
    for frequency, duration in melody:
        play_tone(frequency, duration)

# --- Melodies ---
object_detected_melody = [
    (262, 0.2), (330, 0.2), (392, 0.2), (523, 0.2)
]
edge_detected_melody = [
    (523, 0.2), (392, 0.2), (330, 0.2), (262, 0.2)
]

# --- Main Program ---
try:
    while True:
        # --- Edge Detection (check first) ---
        if GPIO.input(left_edge_pin) == GPIO.LOW or GPIO.input(right_edge_pin) == GPIO.LOW:
            # Edge detected! 
            print("Edge detected!")

            # Stop, reverse, turn, and play melody
            set_motor_speed(motor1_in1, motor1_in2, 0)
            set_motor_speed(motor2_in1, motor2_in2, 0)
            time.sleep(0.5)
            set_motor_speed(motor1_in1, motor1_in2, -50)
            set_motor_speed(motor2_in1, motor2_in2, -50)
            time.sleep(1)
            play_melody(edge_detected_melody)

            # Turn 90 degrees (adjust as needed)
            set_motor_speed(motor1_in1, motor1_in2, -75)  
            set_motor_speed(motor2_in1, motor2_in2, 75)
            time.sleep(0.5)  # Adjust turning duration

            continue  # Go back to the beginning of the loop

        # --- Obstacle Avoidance ---
        clear_path = False
        for angle in range(-60, 61, 20):
            set_servo_angle(servo_channel, angle)
            time.sleep(0.2)
            distance = read_grove_distance(grove_ultrasonic_pin)
            print(f"Angle: {angle}, Distance: {distance:.2f} cm")
            if distance < 30:  # Object detected!
                play_melody(object_detected_melody)
                # Object detected! Stop and maybe move backward
                set_motor_speed(motor1_in1, motor1_in2, 0)
                set_motor_speed(motor2_in1, motor2_in2, 0)
                time.sleep(0.5)
                set_motor_speed(motor1_in1, motor1_in2, -50)
                set_motor_speed(motor2_in1, motor2_in2, -50)
                time.sleep(1)
                break  # Exit the for loop after detecting an object
            else:
                # No object detected, move forward
                set_motor_speed(motor1_in1, motor1_in2, 75)
                set_motor_speed(motor2_in1, motor2_in2, 75)

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    pca.deinit()
    GPIO.cleanup()
    print("Program stopped.")