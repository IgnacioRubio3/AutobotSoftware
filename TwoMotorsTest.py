from gpiozero import PWMOutputDevice, DigitalOutputDevice
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

def drive(left_speed, right_speed):
    """
    Core function to set motor speeds.
    Range: -1.0 (Full Reverse) to 1.0 (Full Forward)
    """
    stby.on()
    
    # Left Motor Logic
    if left_speed > 0:
        ain1.on(); ain2.off(); pwma.value = left_speed
    elif left_speed < 0:
        ain1.off(); ain2.on(); pwma.value = abs(left_speed)
    else:
        ain1.off(); ain2.off(); pwma.value = 0

    # Right Motor Logic
    if right_speed > 0:
        bin1.on(); bin2.off(); pwmb.value = right_speed
    elif right_speed < 0:
        bin1.off(); bin2.on(); pwmb.value = abs(right_speed)
    else:
        bin1.off(); bin2.off(); pwmb.value = 0

try:
    # 1. ONE MOTOR RUNNING IN BOTH DIRECTIONS
    print("Requirement 1: Left motor forward then backward")
    drive(0.5, 0)
    time.sleep(2)
    drive(-0.5, 0)
    time.sleep(2)
    
    print("Requirement 1: Right motor forward then backward")
    drive(0, 0.5)
    time.sleep(2)
    drive(0, -0.5)
    time.sleep(2)

    # 2. BOTH RUNNING IN SAME DIRECTION (Forward and Backward)
    print("Requirement 2: Both motors forward (Straight)")
    drive(0.6, 0.6)
    time.sleep(2)
    
    print("Requirement 2: Both motors backward (Straight)")
    drive(-0.6, -0.6)
    time.sleep(2)

    # 3. BOTH RUNNING IN OPPOSITE DIRECTIONS (Rotations)
    print("Requirement 3: Opposite directions (Spinning Left)")
    drive(-0.5, 0.5)
    time.sleep(2)

    print("Requirement 3: Opposite directions (Spinning Right)")
    drive(0.5, -0.5)
    time.sleep(2)

finally:
    # Always shut down motors at the end
    drive(0, 0)
    stby.off()
    print("Sequence finished. Motors stopped.")