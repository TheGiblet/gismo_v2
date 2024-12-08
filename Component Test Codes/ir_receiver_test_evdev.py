# ir_receiver_test_evdev.py
# Tests the IR receiver using the evdev library.

from evdev import InputDevice, categorize, ecodes
import time

# Replace with the actual path to your input device
device_path = "/dev/input/event0"  # Find the correct eventX using `ls /dev/input/`

try:
    # Create an InputDevice object
    device = InputDevice(device_path)
    print(f"Connected to IR receiver: {device.name}")

    # Grab the device to prevent other programs from accessing it
    device.grab()

    # Read and print IR codes
    for event in device.read_loop():
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            if key_event.keystate == key_event.key_down:  # Only react to key presses
                print("Key pressed:", key_event.keycode)
                # Add your code here to map keycodes to actions
                # ...

except KeyboardInterrupt:
    # Release the device on exit (Ctrl+C)
    device.ungrab()
    print("IR receiver test stopped.")
except FileNotFoundError:
    print(f"Error: Device not found at {device_path}")
    print("Make sure the IR receiver is connected and the device path is correct.")
except Exception as e:
    print(f"Error: {e}")