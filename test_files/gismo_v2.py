import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Create a PCA9685 object.
pca = PCA9685(i2c)

# Set the PWM frequency to 60Hz, a common value for servos and motors.
pca.frequency = 60

# Define the PCA9685 channels connected to the L298N.
# (Because using "INT1" would be far too sensible, wouldn't it?)
INT1 = 7  
INT2 = 8
INT3 = 10
INT4 = 9

# Function to set motor speed.
# Speed ranges from -1 (full reverse) to 1 (full forward).
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

# Test the motors.
# (Feel free to adjust the duration and speed.)

# Forward
set_motor_speed(1, 0.5)  # Left motor at half speed forward
set_motor_speed(2, 0.5)  # Right motor at half speed forward
time.sleep(2)

# Reverse
set_motor_speed(1, -0.5)  # Left motor at half speed reverse
set_motor_speed(2, -0.5)  # Right motor at half speed reverse
time.sleep(2)

# Turn Left (Right motor forward, left motor stopped)
set_motor_speed(1, 0)    # Left motor stop
set_motor_speed(2, 0.5)  # Right motor at half speed forward
time.sleep(2)

# Turn Right (Left motor forward, right motor stopped)
set_motor_speed(1, 0.5)  # Left motor at half speed forward
set_motor_speed(2, 0)    # Right motor stop
time.sleep(2)

# Stop
set_motor_speed(1, 0)
set_motor_speed(2, 0)