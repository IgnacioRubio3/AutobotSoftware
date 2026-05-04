import time
import cv2
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from phase2_linker_s1s6 import load_config, run_s1_to_s6

# --- Hardware Setup ---
# Motor A (Left)
ain1, ain2, pwma = DigitalOutputDevice(27), DigitalOutputDevice(22), PWMOutputDevice(12)
# Motor B (Right)
bin1, bin2, pwmb = DigitalOutputDevice(24), DigitalOutputDevice(25), PWMOutputDevice(13)
stby = DigitalOutputDevice(23)

# --- PID & Control Parameters ---
BASE_SPEED = 0.45    # Constant forward speed (0.0 to 1.0)
KP = 0.40           # How aggressively to steer (Proportional)
KD = 0.05           # Smooths out jitter (Derivative)
last_error = 0.0

def drive(left_speed, right_speed):
    stby.on()
    # Left Motor
    pwma.value = max(0, min(1, abs(left_speed)))
    ain1.value = left_speed > 0
    ain2.value = left_speed < 0
    # Right Motor
    pwmb.value = max(0, min(1, abs(right_speed)))
    bin1.value = right_speed > 0
    bin2.value = right_speed < 0

def main():
    global last_error
    cfg = load_config()
    cap = cv2.VideoCapture(0) # Open Raspberry Pi Camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    frame_id = 0
    print("Robot Live: Following Lane...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break
            
            timestamp = int(time.time() * 1000)
            
            # --- Step 1: Run Vision Pipeline (Stages 1-6) ---
            result = run_s1_to_s6(frame, frame_id, timestamp, cfg)
            error = result.lane_offset.offset # Normalized [-1.0 to 1.0]
            
            # --- Step 2: PID Logic ---
            # Proportional: Current error
            # Derivative: Change in error (prevents overshooting)
            derivative = error - last_error
            steering_correction = (error * KP) + (derivative * KD)
            
            # --- Step 3: Differential Steering ---
            # If error is positive (Robot is too far left):
            # Slow down right motor, speed up left motor to turn right.
            left_motor_speed = BASE_SPEED + steering_correction
            right_motor_speed = BASE_SPEED - steering_correction
            
            drive(left_motor_speed, right_motor_speed)
            
            # --- Step 4: Logging ---
            if frame_id % 10 == 0:
                print(f"ID: {frame_id} | Offset: {error:+.2f} | Steering: {steering_correction:+.2f}")
            
            last_error = error
            frame_id += 1

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        drive(0, 0)
        stby.off()
        cap.release()

if __name__ == "__main__":
    main()