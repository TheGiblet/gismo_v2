# tilt_sensor_test.py
# Tests the KY-020 Tilt Sensor module connected to GPIO 8.

import RPi.GPIO as GPIO
import time

# Define the GPIO pin for the tilt sensor
tilt_pin = 20  # Changed to GPIO 20

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set tilt sensor pin as input
GPIO.setup(tilt_pin, GPIO.IN)

try:
    while True:
        if GPIO.input(tilt_pin) == GPIO.HIGH:  # Tilt detected
            print("Tilt detected!")
        else:
            print("No tilt")

        time.sleep(0.2)  # Adjust delay as needed

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    GPIO.cleanup()
    print("Tilt sensor test stopped.")