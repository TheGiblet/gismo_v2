# rgb_led_test.py
# Tests the KY-016 RGB LED module connected to GPIO 8, 11, and 14.

import RPi.GPIO as GPIO
import time

# Define the GPIO pins for the RGB LED
red_pin = 8
green_pin = 11
blue_pin = 14

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set LED pins as output
GPIO.setup(red_pin, GPIO.OUT)
GPIO.setup(green_pin, GPIO.OUT)
GPIO.setup(blue_pin, GPIO.OUT)

try:
    while True:
        # Cycle through red, green, and blue
        GPIO.output(red_pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(red_pin, GPIO.LOW)

        GPIO.output(green_pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(green_pin, GPIO.LOW)

        GPIO.output(blue_pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(blue_pin, GPIO.LOW)

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    GPIO.cleanup()
    print("RGB LED test stopped.")