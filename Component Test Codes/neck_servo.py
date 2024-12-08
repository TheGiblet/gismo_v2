# servo_control.py
# Controls a 9g servo connected to channel 4 of the PCA9685 PWM driver.
# This servo acts as the robot's neck, rotating an ultrasonic sensor horizontally.

import time
from board import SCL, SDA  # Import I2C pins
import busio

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

# Create a PCA9685 class instance.
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz, suitable for most servos.
pca.frequency = 60

# Define the servo channel (using channel 4)
servo_channel = 4  

# Function to set servo angle
def set_servo_angle(channel, angle):
  """Sets the angle of a servo motor for horizontal rotation.

  Args:
      channel: The PCA9685 channel the servo is connected to.
      angle: The desired angle in degrees (-60 to +60).
  """
  if angle < -60:
      angle = -60  # Ensure angle is not less than -60
  elif angle > 60:
      angle = 60   # Ensure angle is not greater than 60

  # Calculate the pulse width for the servo. 1500 is the center position.
  # This assumes a 120-degree range of motion (-60 to +60).
  pulse = int((angle + 60) / 120 * (65535 - 1500) + 1500)  
  pca.channels[channel].duty_cycle = pulse  # Set the duty cycle for the channel


# Function to read ultrasonic distance (replace with your actual implementation)
def read_ultrasonic_distance():
  """Reads the distance from the ultrasonic sensor.

  Returns:
      The distance in centimeters (or appropriate units).
  """
  # Replace this with your actual code to read from the ultrasonic sensor
  # This is just a placeholder
  # You'll need to use the `RPi.GPIO` library and the appropriate trigger/echo pins
  # to interact with the ultrasonic sensor.
  return 20  # Example distance


try:
  while True:
    # Scan from -60 to +60 degrees
    for angle in range(-60, 61, 10):  # Step by 10 degrees
        set_servo_angle(servo_channel, angle)
        time.sleep(0.2)  # Adjust delay as needed

        # Read distance from the ultrasonic sensor
        distance = read_ultrasonic_distance() 
        print(f"Angle: {angle}, Distance: {distance}")

    # Scan back from +60 to -60 degrees
    for angle in range(60, -61, -10):
        set_servo_angle(servo_channel, angle)
        time.sleep(0.2)

        # Read distance from the ultrasonic sensor
        distance = read_ultrasonic_distance()
        print(f"Angle: {angle}, Distance: {distance}")

except KeyboardInterrupt:
  # Clean up on exit (Ctrl+C)
  pca.deinit()  # Deinitialize the PCA9685
  print("Servo control stopped.")