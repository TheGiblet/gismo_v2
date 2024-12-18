# edge_sensor_test.py
# Tests two KY-033 Edge Detection Modules connected to GPIO 12 and 13.

import RPi.GPIO as GPIO
import time

# Define GPIO pins for the edge sensors
left_edge_pin = 12
right_edge_pin = 13

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set edge sensor pins as input with pull-up resistors
GPIO.setup(left_edge_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(right_edge_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        # Check left edge sensor
        if GPIO.input(left_edge_pin) == GPIO.LOW:
            print("Left edge detected!")

        # Check right edge sensor
        if GPIO.input(right_edge_pin) == GPIO.LOW:
            print("Right edge detected!")

        time.sleep(0.1)  # Add a small delay

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    GPIO.cleanup()
    print("Edge sensor test stopped.")