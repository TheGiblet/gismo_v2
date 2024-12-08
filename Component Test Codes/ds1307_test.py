# ds1307_test.py
# Tests the DS1307 Real-Time Clock (RTC) module connected to the I2C bus.

import time
import board
import busio
import adafruit_ds1307

# Initialize I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Create an instance of the DS1307 class.
rtc = adafruit_ds1307.DS1307(i2c)

# --- Optional: Set the time (only needs to be done once) ---
# Uncomment the following lines to set the time initially
from datetime import datetime
t = datetime.now()  # Get the current time
rtc.datetime = t    # Set the RTC time

# --- Read and display the time ---
while True:
    current_time = rtc.datetime
    print("Current time: {}/{}/{} {}:{}:{}".format(
        current_time.tm_year,
        current_time.tm_mon,
        current_time.tm_mday,
        current_time.tm_hour,
        current_time.tm_min,
        current_time.tm_sec
    ))
    time.sleep(1)  # Update every second