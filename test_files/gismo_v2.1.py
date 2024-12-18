import time
import board
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO

# Initialize I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Create a PCA9685 object.
pca = PCA9685(i2c)

# Set the PWM frequency to 60Hz.
pca.frequency = 60

# Define the PCA9685 channels connected to the L298N.
INT1 = 7  
INT2 = 8
INT3 = 10
INT4 = 9

# Define GPIO pins for line tracking sensors
left_sensor_pin = 12
right_sensor_pin = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_sensor_pin, GPIO.IN) 
GPIO.setup(right_sensor_pin, GPIO.IN)

# Function to set motor speed.
def set_motor_speed(motor, speed):
  """Sets the speed of a motor.

  Args:
    motor: 1 for left motor, 2 for right motor.
    speed: -1 to 1, where -1 is full reverse, 1 is full forward.
  """
  if motor == 1:
    if speed > 0:  # Forward
      pca.channels[INT1].duty_cycle = int(speed * 65535)
      pca.channels[INT2].duty_cycle = 0
    elif speed < 0:  # Reverse
      pca.channels[INT1].duty_cycle = 0
      pca.channels[INT2].duty_cycle = int(-speed * 65535)
    else:  # Stop
      pca.channels[INT1].duty_cycle = 0
      pca.channels[INT2].duty_cycle = 0
  elif motor == 2:
    if speed > 0:  # Forward
      pca.channels[INT3].duty_cycle = int(speed * 65535)
      pca.channels[INT4].duty_cycle = 0
    elif speed < 0:  # Reverse
      pca.channels[INT3].duty_cycle = 0
      pca.channels[INT4].duty_cycle = int(-speed * 65535)
    else:  # Stop
      pca.channels[INT3].duty_cycle = 0
      pca.channels[INT4].duty_cycle = 0

# Main program loop
try:
    while True:
        left_sensor_value = GPIO.input(left_sensor_pin)
        right_sensor_value = GPIO.input(right_sensor_pin)

        if left_sensor_value == 1 or right_sensor_value == 1:  # Edge detected
            print("Edge detected! Stopping.")
            set_motor_speed(1, 0)  # Stop left motor
            set_motor_speed(2, 0)  # Stop right motor
        else:
            # No edge detected, move forward
            set_motor_speed(1, 0.5)  # Left motor forward
            set_motor_speed(2, 0.5)  # Right motor forward

        time.sleep(0.1)  # Adjust delay as needed

except KeyboardInterrupt:
    # Stop motors on Ctrl+C
    set_motor_speed(1, 0)
    set_motor_speed(2, 0)
    pca.deinit()
    GPIO.cleanup()