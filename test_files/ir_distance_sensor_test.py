# ir_distance_sensor_test.py
# Tests the KY-032 Infrared Distance Sensor module connected to GPIO 14.

import RPi.GPIO as GPIO
import time

# Define the GPIO pin for the IR distance sensor
ir_distance_pin = 14

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set the sensor pin as input
GPIO.setup(ir_distance_pin, GPIO.IN)

try:
    while True:
        if GPIO.input(ir_distance_pin) == GPIO.LOW:
            print("Object detected!")
        else:
            print("No object detected.")

        time.sleep(0.2)  # Adjust the delay as needed

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    GPIO.cleanup()
    print("IR distance sensor test stopped.")