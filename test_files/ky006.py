import time
import RPi.GPIO as GPIO

# Define the GPIO pin for the buzzer.
BUZZER_PIN = 18

# Function to play a tone.
def play_tone(pin, frequency, duration):
  """Plays a tone on the specified pin.

  Args:
    pin: The GPIO pin connected to the buzzer.
    frequency: The frequency of the tone in Hz.
    duration: The duration of the tone in seconds.
  """
  GPIO.setup(pin, GPIO.OUT)
  pwm = GPIO.PWM(pin, frequency)
  pwm.start(50)  # 50% duty cycle
  time.sleep(duration)
  pwm.stop()

# Initialize GPIO
GPIO.setmode(GPIO.BCM)

# Test the buzzer.
while True:
  # Play a simple tune.
  play_tone(BUZZER_PIN, 262, 0.5)  # Middle C
  time.sleep(0.1)
  play_tone(BUZZER_PIN, 330, 0.5)  # E
  time.sleep(0.1)
  play_tone(BUZZER_PIN, 392, 0.5)  # G
  time.sleep(0.1)
  play_tone(BUZZER_PIN, 523, 0.5)  # High C
  time.sleep(0.5)