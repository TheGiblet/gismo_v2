# camera_test.py
# Captures a still image from the Pi Camera using libcamera and saves it 
# with a timestamped filename.

import datetime
import subprocess

def take_picture(filename_prefix):
    """Captures a picture with a timestamped filename."""
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = f"{filename_prefix}_{timestamp}.jpg"
    try:
        # Use libcamera-still to capture a still image
        subprocess.run(["libcamera-still", "-o", filename], check=True)
        print(f"Picture saved to {filename}")
    except subprocess.CalledProcessError as e:
        print(f"Error taking picture: {e}")

# Example usage
if __name__ == "__main__":
    take_picture("gismo_image")  # Captures an image named "gismo_image_YYYYMMDD-HHMMSS.jpg"