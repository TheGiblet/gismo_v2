import time
import RPi.GPIO as GPIO

# Define the GPIO pin for the sound sensor.
SOUND_SENSOR_PIN = 21

# Function to read sound level.
def get_sound_level(pin):
  """Reads the sound level from the sensor.

  Args:
    pin: The GPIO pin connected to the sound sensor.

  Returns:
    True if sound is detected, False otherwise.
  """
  GPIO.setup(pin, GPIO.IN)
  sound_detected = GPIO.input(pin)  # High signal when sound is detected
  return sound_detected

# Initialize GPIO
GPIO.setmode(GPIO.BCM)

# Test the sound sensor.
while True:
  sound_detected = get_sound_level(SOUND_SENSOR_PIN)

  if sound_detected:
    print("Sound detected!")
  else:
    print("Silence...")

  time.sleep(0.1)  # Pause between readings