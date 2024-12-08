# button_test.py
# Tests the KY-004 button module connected to GPIO 27 on the Raspberry Pi.

import RPi.GPIO as GPIO
import time

# Define the button pin
button_pin = 27

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set button pin as input with pull-up resistor
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        if GPIO.input(button_pin) == GPIO.LOW:  # Button is pressed
            print("Button pressed!")
        # else:
        #     print("Button not pressed")  # You can uncomment this for more verbose output

        time.sleep(0.1)  # Add a small delay

except KeyboardInterrupt:
    # Clean up on exit (Ctrl+C)
    GPIO.cleanup()
    print("Button test stopped.")