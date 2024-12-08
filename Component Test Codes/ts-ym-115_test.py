# ts-ym-115_test.py
# Tests the TS-YM-115 sound detection sensor module.

import time
import RPi.GPIO as GPIO

# Define the GPIO pin for the sensor's digital output (DO) pin
sound_sensor_pin = 20  # You can change this to any available GPIO pin

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set the sensor pin as input
GPIO.setup(sound_sensor_pin, GPIO.IN)

try:
    while True:
        if GPIO.input(sound_sensor_pin) == GPIO.HIGH:
            print("Sound detected!")
        else:
            print("No sound")

        time.sleep(0.1)  # Adjust the delay as needed

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    GPIO.cleanup()
    print("Sound sensor test stopped.")