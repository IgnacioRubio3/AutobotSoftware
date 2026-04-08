from gpiozero import PWMOutputDevice, DigitalOutputDevice
from pynput import keyboard
import time

# --- Hardware Setup (Same Pins) ---
pwma = PWMOutputDevice(12)
ain1 = DigitalOutputDevice(27)
ain2 = DigitalOutputDevice(22)
stby = DigitalOutputDevice(23)

# --- Logic Variables ---
current_speed = 0.0
increment = 0.05  # How fast it ramps up (0.0 to 1.0)
active_key = None

def update_motor(speed):
    """Applies the speed to the hardware"""
    stby.on()
    if speed > 0:
        ain1.on()
        ain2.off()
        pwma.value = min(speed, 1.0) # Cap at 1.0
    elif speed < 0:
        ain1.off()
        ain2.on()
        pwma.value = min(abs(speed), 1.0)
    else:
        ain1.off()
        ain2.off()
        pwma.value = 0

def on_press(key):
    global active_key
    try:
        if key.char == 'w':
            active_key = 'w'
        elif key.char == 's':
            active_key = 's'
    except AttributeError:
        pass

def on_release(key):
    global active_key, current_speed
    active_key = None
    # Reset speed on release or stop immediately
    current_speed = 0
    update_motor(0)
    if key == keyboard.Key.esc:
        return False # Stop listener

# --- Main Loop ---
print("Hold 'W' to go forward, 'S' to go backward. ESC to quit.")

# Start keyboard listener in the background
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

try:
    while listener.running:
        if active_key == 'w':
            current_speed += increment
            if current_speed > 1.0: current_speed = 1.0
        elif active_key == 's':
            current_speed -= increment
            if current_speed < -1.0: current_speed = -1.0
        else:
            current_speed = 0
            
        update_motor(current_speed)
        
        # Small delay to control the 'ramp up' speed
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    update_motor(0)
    stby.off()
    print("\nControls Closed.")