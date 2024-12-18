import time
import RPi.GPIO as GPIO

# Define the GPIO pins for the edge sensors.
#LEFT_EDGE_PIN = 12
RIGHT_EDGE_PIN = 13

# Function to detect an edge.
def detect_edge(pin):
  """Detects an edge (drop-off) on the specified pin.

  Args:
    pin: The GPIO pin connected to the edge sensor.

  Returns:
    True if an edge is detected, False otherwise.
  """
  GPIO.setup(pin, GPIO.IN)
  edge_detected = not GPIO.input(pin)  # Active low signal
  return edge_detected

# Initialize GPIO
GPIO.setmode(GPIO.BCM)

# Test the edge sensors.
while True:
#  left_edge = detect_edge(LEFT_EDGE_PIN)
  right_edge = detect_edge(RIGHT_EDGE_PIN)

#  if left_edge:
#    print("Left edge detected!")
  if right_edge:
    print("Right edge detected!")

  time.sleep(0.1)  # Pause between readings