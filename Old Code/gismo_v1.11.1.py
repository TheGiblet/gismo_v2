# gismo_v1.11.1.py
# Combines motor control, neck servo, Grove Ultrasonic sensor, edge detection,
# buzzer feedback with melodies, RGB LED feedback (on PCA9685), touch sensor
# interaction, IR distance sensor for reverse obstacle avoidance, button
# control for debugging, camera functionality, and IR remote control (using evdev).
# The robot prioritizes edge detection, scans for a clear path, avoids objects,
# backs up and turns when an edge is detected, uses the buzzer to play different
# melodies, provides visual feedback with the RGB LED, shakes when touched,
# displays a smiling face on the OLED normally and a sad face when an object
# or edge is detected, uses the IR distance sensor to prevent backing into
# obstacles, allows button-triggered debugging actions (running sensor test
# code files), takes a picture when an object is detected, and responds to
# commands from an IR remote.

import datetime  # Import the datetime module for timestamps
import time
from board import SCL, SDA
import busio
import RPi.GPIO as GPIO
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
from adafruit_shell import Shell

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# --- evdev for IR Remote Control ---
from evdev import InputDevice, categorize, ecodes

# Replace with the actual path to your input device
device_path = "/dev/input/eventX"  # Find the correct eventX using `ls /dev/input/`

# --- I2C Setup for PCA9685 and Display ---
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# --- Display Setup ---
display = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c_bus)

# --- GPIO Setup for Sensors, Buzzer, and Touch Sensor ---
grove_ultrasonic_pin = 25
left_edge_pin = 12
right_edge_pin = 13
buzzer_pin = 18
touch_sensor = 9
ir_distance_pin = 26  # KY-032 on GPIO 26
button_pin = 27
tilt_pin = 20  # KY-020 on GPIO 20

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_edge_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(right_edge_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buzzer_pin, GPIO.OUT)
GPIO.setup(touch_sensor, GPIO.IN)
GPIO.setup(ir_distance_pin, GPIO.IN)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(tilt_pin, GPIO.IN)

# --- Motor, Servo, and RGB LED Channels ---
motor1_in1 = 0
motor1_in2 = 1
motor2_in1 = 2
motor2_in2 = 3
servo_channel = 4
red_channel = 7
green_channel = 8
blue_channel = 9

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

# --- Function to take a picture with timestamp ---
def take_picture(filename_prefix):
    """Captures a picture with a timestamped filename."""
    import subprocess  # Import subprocess here
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = f"{filename_prefix}_{timestamp}.jpg"
    try:
        # Use libcamera-still to capture a still image
        subprocess.run(["libcamera-still", "-o", filename], check=True)
        print(f"Picture saved to {filename}")
    except subprocess.CalledProcessError as e:
        print(f"Error taking picture: {e}")

# --- Melodies ---
object_detected_melody = [
    (262, 0.2), (330, 0.2), (392, 0.2), (523, 0.2)
]
edge_detected_melody = [
    (523, 0.2), (392, 0.2), (330, 0.2), (262, 0.2)
]

# --- Initialize Shell ---
shell = Shell()

# --- Main Program ---
try:
    # Create an InputDevice object
    device = InputDevice(device_path)
    print(f"Connected to IR receiver: {device.name}")

    # Grab the device to prevent other programs from accessing it
    device.grab()

    while True:
        # --- Button for Debugging ---
        if GPIO.input(button_pin) == GPIO.LOW:
            print("Button pressed! Running test code...")

            # --- Run the test code files ---
            print("Running edge_sensor_test.py...")
            shell.run_command("python edge_sensor_test.py") 

            print("Running grove_ultrasonic_test.py...")
            shell.run_command("python grove_ultrasonic_test.py")

            print("Running buzzer_test.py...")
            shell.run_command("python buzzer_test.py")

            print("Running touch_sensor_test.py...")
            shell.run_command("python touch_sensor_test.py")

            print("Running ir_distance_sensor_test.py...")
            shell.run_command("python ir_distance_sensor_test.py")

            print("Running ds1307_test.py...")
            shell.run_command("python ds1307_test.py")

            print("Running ky034_test.py...")
            shell.run_command("python ky034_test.py")

            print("Running rgb_led_test.py...")
            shell.run_command("python rgb_led_test.py")

            time.sleep(5)  # Add a delay to avoid multiple triggers

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

                # Object detected! Stop, take picture, and move backward
                set_motor_speed(motor1_in1, motor1_in2, 0)
                set_motor_speed(motor2_in1, motor2_in2, 0)
                take_picture("object_detected")  # Capture and save the image
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

        # --- Reverse Obstacle Detection ---
        if GPIO.input(ir_distance_pin) == GPIO.LOW:  # Object detected behind
            print("Object detected behind!")
            # Stop or take other action to avoid backing into the object
            set_motor_speed(motor1_in1, motor1_in2, 0)  # Stop motors
            set_motor_speed(motor2_in1, motor2_in2, 0)
            time.sleep(0.5)
            # You might want to add code to turn or move forward slightly
            # to clear the obstacle behind

        # --- Tilt Detection ---
        if GPIO.input(tilt_pin) == GPIO.HIGH:  # Tilt detected
            print("Tilt detected!")
            # Respond to the tilt
            set_motor_speed(motor1_in1, motor1_in2, 0)  # Stop motors
            set_motor_speed(motor2_in1, motor2_in2, 0)
            play_tone(1000, 0.5)  # Play a warning tone
            draw_face("sad")  # Display a sad face

        # --- IR Remote Control (using evdev) ---
        for event in device.read_loop():
            if event.type == ecodes.EV_KEY:
                key_event = categorize(event)
                if key_event.keystate == key_event.key_down:
                    print("Key pressed:", key_event.keycode)
                    # Map the received keycode to actions
                    if key_event.keycode == "KEY_UP":
                        print("Moving forward")
                        set_motor_speed(motor1_in1, motor1_in2, 75)
                        set_motor_speed(motor2_in1, motor2_in2, 75)
                    elif key_event.keycode == "KEY_DOWN":
                        print("Moving backward")
                        set_motor_speed(motor1_in1, motor1_in2, -50)
                        set_motor_speed(motor2_in1, motor2_in2, -50)
                    elif key_event.keycode == "KEY_LEFT":
                        print("Turning left")
                        set_motor_speed(motor1_in1, motor1_in2, 50)
                        set_motor_speed(motor2_in1, motor2_in2, -50)
                    elif key_event.keycode == "KEY_RIGHT":
                        print("Turning right")
                        set_motor_speed(motor1_in1, motor1_in2, -50)
                        set_motor_speed(motor2_in1, motor2_in2, 50)
                    elif key_event.keycode == "KEY_OK":
                        print("Stopping")
                        set_motor_speed(motor1_in1, motor1_in2, 0)
                        set_motor_speed(motor2_in1, motor2_in2, 0)
                    # ... (add more keycode mappings)

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    # Explicitly stop the motors
    set_motor_speed(motor1_in1, motor1_in2, 0)
    set_motor_speed(motor2_in1, motor2_in2, 0)

    pca.deinit()  # Deinitialize the PCA9685
    GPIO.cleanup()
    device.ungrab()  # Release the IR receiver device
    print("Program stopped.")
except FileNotFoundError:
    print(f"Error: Device not found at {device_path}")
    print("Make sure the IR receiver is connected and the device path is correct.")
except Exception as e:
    print(f"Error: {e}")