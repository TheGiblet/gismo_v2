# grove_ultrasonic_test.py
# Tests the Grove Ultrasonic Distance Sensor connected to GPIO 25.

import time
import RPi.GPIO as GPIO

# Define the GPIO pin for the sensor
grove_ultrasonic_pin = 25

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Function to read distance from the Grove Ultrasonic Sensor
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

try:
    while True:
        distance = read_grove_distance(grove_ultrasonic_pin)
        print(f"Distance: {distance:.2f} cm")
        time.sleep(1)  # Adjust delay as needed

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    GPIO.cleanup()
    print("Grove Ultrasonic test stopped.")