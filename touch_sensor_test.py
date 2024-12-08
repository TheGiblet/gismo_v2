# touch_sensor_test.py
# Tests the TTP223B Capacitive Touch Sensor module connected to GPIO 17.

import RPi.GPIO as GPIO
import time

# Define the GPIO pin for the touch sensor
touch_pin = 9

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set touch sensor pin as input
GPIO.setup(touch_pin, GPIO.IN)  # No pull-up needed, the module has an internal pull-up

try:
    while True:
        if GPIO.input(touch_pin) == GPIO.HIGH:
            print("Touch detected!")
        else:
            print("No touch")

        time.sleep(0.2)  # Add a small delay

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    GPIO.cleanup()
    print("Touch sensor test stopped.")