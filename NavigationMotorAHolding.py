from gpiozero import PWMOutputDevice, DigitalOutputDevice
import readchar
import time

# --- Hardware Setup (Based on your Pinout) ---
# Physical 32 -> GPIO 12 (PWMA)
# Physical 13 -> GPIO 27 (AIN1)
# Physical 15 -> GPIO 22 (AIN2)
# Physical 16 -> GPIO 23 (STBY)

pwma = PWMOutputDevice(12)
ain1 = DigitalOutputDevice(27)
ain2 = DigitalOutputDevice(22)
stby = DigitalOutputDevice(23)

# Settings
SPEED_STEP = 0.1  # How much to increment speed per key press
current_speed = 0.0

def update_motor(speed):
    """Updates motor hardware based on speed (-1.0 to 1.0)"""
    stby.on()
    if speed > 0:
        ain1.on()
        ain2.off()
        pwma.value = min(speed, 1.0)
    elif speed < 0:
        ain1.off()
        ain2.on()
        pwma.value = min(abs(speed), 1.0)
    else:
        ain1.off()
        ain2.off()
        pwma.value = 0

print("--- MOTOR CONTROL READY ---")
print("Hold 'W' for Forward, 'S' for Backward.")
print("Press 'Q' to Emergency Stop and Quit.")

try:
    while True:
        # Read a single key press
        key = readchar.readkey().lower()

        if key == 'w':
            current_speed = min(current_speed + SPEED_STEP, 1.0)
            update_motor(current_speed)
            print(f"Moving Forward: {current_speed:.1f}", end="\r")
            
        elif key == 's':
            current_speed = max(current_speed - SPEED_STEP, -1.0)
            update_motor(current_speed)
            print(f"Moving Backward: {abs(current_speed):.1f}", end="\r")
            
        elif key == 'q':
            print("\nEmergency Stop Triggered.")
            break
        
        else:
            # If any other key is pressed, stop the motor
            current_speed = 0.0
            update_motor(0)
            print("Motor Stopped.        ", end="\r")

        # Small sleep to keep the CPU happy
        time.sleep(0.05)
        
except KeyboardInterrupt:
    pass
finally:
    update_motor(0)
    stby.off()
    print("\nGPIO Cleaned Up. Exiting.")
