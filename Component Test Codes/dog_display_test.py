# display_dog_face.py
# Displays a dog face image (dog_face.bmp) on the SSD1306 OLED display.

import time
import board
import busio
import adafruit_ssd1306
from adafruit_framebuf import FrameBuffer, bitmap_font  # Import bitmap_font

# Define the I2C bus and display dimensions
i2c = busio.I2C(board.SCL, board.SDA)
display = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

# Clear the display
display.fill(0)
display.show()

# Load the monochrome bitmap (make sure dog_face.bmp is in the same directory)
with open("dog_face.bmp", "rb") as f:
    bitmap = bitmap_font.MonochromeBitmap.load(f)  # Access MonochromeBitmap through bitmap_font

# Create a frame buffer
fb = bitmap_font.FrameBuffer(
    bitmap,
    display.width,
    display.height,
    buf_format=bitmap_font.MonochromeBitmap.FORMAT_1,
)

# Display the image
display.image(fb)
display.show()