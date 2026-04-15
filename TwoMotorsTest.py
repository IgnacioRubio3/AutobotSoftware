from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
import readchar
import time

# --- Hardware Setup ---
# Motor A (Left)
ain1 = DigitalOutputDevice(27)
ain2 = DigitalOutputDevice(22)
pwma = PWMOutputDevice(12)

# Motor B (Right)
bin1 = DigitalOutputDevice(24)
bin2 = DigitalOutputDevice(25)
pwmb = PWMOutputDevice(13)

# Common Standby
stby = DigitalOutputDevice(23)

# Encoders
encoder_l = RotaryEncoder(16, 19)
encoder_r = RotaryEncoder(20, 21)

# Logic Variables
current_speed = 0.0
increment = 0.1 

def update_motors(speed):
    """Sets speed for both motors (-1.0 to 1.0)"""
    stby.on()
    
    # Motor A Logic
    if speed > 0:
        ain1.on(); ain2.off(); pwma.value = speed
        bin1.on(); bin2.off(); pwmb.value = speed
    elif speed < 0:
        ain1.off(); ain2.on(); pwma.value = abs(speed)
        bin1.off(); bin2.on(); pwmb.value = abs(speed)
    else:
        ain1.off(); ain2.off(); pwma.value = 0
        bin1.off(); bin2.off(); pwmb.value = 0

print("--- DUAL MOTOR CONTROL ---")
print("Hold 'W' (Forward), 'S' (Backward), 'Q' (Quit)")

try:
    while True:
        # Use readchar for headless terminal compatibility
        key = readchar.readkey().lower()

        if key == 'w':
            current_speed = min(current_speed + increment, 1.0)
            update_motors(current_speed)
            print(f"Moving Forward | L: {encoder_l.steps} R: {encoder_r.steps}", end="\r")
        
        elif key == 's':
            current_speed = max(current_speed - increment, -1.0)
            update_motors(current_speed)
            print(f"Moving Backward | L: {encoder_l.steps} R: {encoder_r.steps}", end="\r")
        
        elif key == 'q':
            print("\nShutting down...")
            break
        
        else:
            # Neutral stop if other keys pressed
            current_speed = 0
            update_motors(0)
            print("Motors Stopped.                          ", end="\r")

        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    update_motors(0)
    stby.off()
    print("\nSafety Shutdown Complete.")