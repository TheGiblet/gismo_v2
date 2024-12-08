# gismo_v1.5.1.py
# Combines motor control, neck servo, Grove Ultrasonic sensor, edge detection,
# buzzer feedback with melodies, RGB LED feedback (now on PCA9685), and 
# touch sensor interaction. The robot prioritizes edge detection, scans for a 
# clear path, avoids objects, backs up and turns when an edge is detected, 
# uses the buzzer to play different melodies, provides visual feedback with 
# the RGB LED, shakes when touched, and displays a smiling face on the OLED 
# normally and a sad face when an object or edge is detected.

import time
from board import SCL, SDA
import busio
import RPi.GPIO as GPIO
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

# Import 1  the PCA9685 module.
from adafruit_pca9685 import PCA9685

# --- I2C Setup for PCA9685 and Display ---
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# --- Display Setup ---
display = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c_bus)

# --- GPIO Setup for Ultrasonic Sensor, Edge Sensors, and Buzzer ---
grove_ultrasonic_pin = 25
left_edge_pin = 12
right_edge_pin = 13
buzzer_pin = 18
touch_sensor = 9  # Touch sensor is on GPIO 9

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_edge_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(right_edge_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buzzer_pin, GPIO.OUT)
GPIO.setup(touch_sensor, GPIO.IN)

# --- Motor, Servo, and RGB LED Channels ---
motor1_in1 = 0
motor1_in2 = 1
motor2_in1 = 2
motor2_in2 = 3
servo_channel = 4
red_channel = 7   # PCA9685 channel for red
green_channel = 8  # PCA9685 channel for green
blue_channel = 9  # PCA9685 channel for blue

# --- Touch Sensor Variables ---
touch_sensor_state = 0
last_touch_time = 0
debounce_delay = 0.2  # Adjust this value as needed

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

def draw_face(expression="happy"):
    """Draws a face on the OLED display."""
    # Clear the display
    display.fill(0)
    display.show()

    # Create blank image for drawing
    image = Image.new("1", (display.width, display.height))
    draw = ImageDraw.Draw(image)

    # Draw the face elements
    draw.ellipse((20, 10, 108, 54), outline=255, fill=0)  # Head
    draw.ellipse((35, 20, 45, 30), outline=255, fill=255)  # Left eye
    draw.ellipse((80, 20, 90, 30), outline=255, fill=255)  # Right eye

    if expression == "happy":
        draw.arc((35, 35, 90, 50), 0, 180, fill=255)  # Smile
    elif expression == "sad":
        draw.arc((35, 35, 90, 50), 180, 0, fill=255)  # Frown

    # Display the image
    display.image(image)
    display.show()

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
        if GPIO.input(left_edge_pin) == GPIO.HIGH or GPIO.input(right_edge_pin) == GPIO.HIGH:
            # Edge detected! 
            print("Edge detected!")

            # Visual feedback: Turn on RED LED and draw sad face
            pca.channels[red_channel].duty_cycle = 65535  # Turn on red LED
            draw_face("sad")

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

            # Turn off RED LED
            pca.channels[red_channel].duty_cycle = 0 

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

                # Visual feedback: Turn on GREEN LED and draw sad face
                pca.channels[green_channel].duty_cycle = 65535  # Turn on green LED
                draw_face("sad")

                # Object detected! Stop and maybe move backward
                set_motor_speed(motor1_in1, motor1_in2, 0)
                set_motor_speed(motor2_in1, motor2_in2, 0)
                time.sleep(0.5)
                set_motor_speed(motor1_in1, motor1_in2, -50)
                set_motor_speed(motor2_in1, motor2_in2, -50)
                time.sleep(1)

                # Turn off GREEN LED
                pca.channels[green_channel].duty_cycle = 0
                break  # Exit the for loop after detecting an object
            else:
                # No object detected, move forward

                # Visual feedback: Turn on BLUE LED and draw happy face
                pca.channels[blue_channel].duty_cycle = 65535  # Turn on blue LED
                draw_face("happy")

                set_motor_speed(motor1_in1, motor1_in2, 75)
                set_motor_speed(motor2_in1, motor2_in2, 75)

                # Turn off BLUE LED
                pca.channels[blue_channel].duty_cycle = 0

        # --- Touch Sensor ---
        if GPIO.input(touch_sensor) == 0 and touch_sensor_state == 0:
            if time.time() - last_touch_time > debounce_delay:
                print("Touch detected! Shaking...")

                # Shake the robot (adjust speed and duration as needed)
                for _ in range(5):
                    set_motor_speed(motor1_in1, motor1_in2, 75)
                    set_motor_speed(motor2_in1, motor2_in2, -75)
                    time.sleep(0.2)
                    set_motor_speed(motor1_in1, motor1_in2, -75)
                    set_motor_speed(motor2_in1, motor2_in2, 75)
                    time.sleep(0.2)

                # Stop the motors after shaking
                set_motor_speed(motor1_in1, motor1_in2, 0)
                set_motor_speed(motor2_in1, motor2_in2, 0)

                touch_sensor_state = 1
                last_touch_time = time.time()
        elif GPIO.input(touch_sensor) == 1:
            touch_sensor_state = 0

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    # Explicitly stop the motors
    set_motor_speed(motor1_in1, motor1_in2, 0)
    set_motor_speed(motor2_in1, motor2_in2, 0)

    pca.deinit()  # Deinitialize the PCA9685
    GPIO.cleanup()
    print("Program stopped.")