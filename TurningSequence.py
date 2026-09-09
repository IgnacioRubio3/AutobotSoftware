import pigpio
import time
import logging

from MikeBigStuff import System

# =============================================================================
# Logging
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("pipeline")

# =============================================================================
# Motor Control Parameters
# =============================================================================
BASE_SPEED = 0.45   # Constant forward speed (0.0 to 1.0)
KP = 0.40   # Proportional gain
KD = 0.05   # Derivative gain;smooths correction jitter

# Soft-start: seconds to linearly ramp from 0 -> BASE_SPEED any time driving
# resumes from a full stop (segment start, or a "stop" drive_state ending).
# Targets motor inrush current at the moment of the speed jump, not just
# steady-state draw -- lowering BASE_SPEED alone doesn't address this.
RAMP_SECONDS = 0.75

_last_error: float = 0.0


# =============================================================================
# Hardware Setup
# =============================================================================
pi = pigpio.pi()

# Motor A (Left) — TB6612 AIN side
_ain1 = 24
_ain2 = 25
_pwma = 13

# Motor B (Right) — TB6612 BIN side
_bin1 = 27
_bin2 = 22
_pwmb = 12
_stby = 23

pi.set_mode(_ain1, pigpio.OUTPUT)
pi.set_mode(_ain2, pigpio.OUTPUT)
pi.set_mode(_bin1, pigpio.OUTPUT)
pi.set_mode(_bin2, pigpio.OUTPUT)
pi.set_mode(_stby, pigpio.OUTPUT)

def _drive(left_speed: float, right_speed: float) -> None:
    """
    Drive the robot with specified left and right motor speeds 
    using the TB6612 motor driver.
    """
    right_speed = -right_speed  # invert if right motor
    pi.write(_stby, 1)

    # Left
    spd_l = int(max(0.0, min(1.0, abs(left_speed))) * 1000000)
    pi.hardware_PWM(_pwma, 1000, spd_l)
    pi.write(_ain1, 1 if left_speed < 0 else 0)
    pi.write(_ain2, 1 if left_speed > 0 else 0)

    # Right
    spd_r = int(max(0.0, min(1.0, abs(right_speed))) * 1000000)
    pi.hardware_PWM(_pwmb, 1000, spd_r)
    pi.write(_bin1, 1 if right_speed > 0 else 0)
    pi.write(_bin2, 1 if right_speed < 0 else 0)

def main() -> None:
    log.info("Starting Navilott Pipeline")

    s = System()

    s.wait_for_start()
    s.run_countdown()

    try:
        # =================================================================
        # Motor Control Sequence with Manual Button Triggers
        # =================================================================
        
        # Step 1
        _drive(0.45, 0.45)
        time.sleep(2.26)
        _drive(0.0, 0.0)

        # Step 2
        s.wait_for_start()
        s.run_countdown()
        _drive(0.45, 0.45)
        time.sleep(1.263)
        _drive(0.0, 0.0)

        # Step 3
        s.wait_for_start()
        s.run_countdown()
        _drive(0.45, 0.0)
        time.sleep(1.7)
        _drive(0.0, 0.0)

        # Step 4
        s.wait_for_start()
        s.run_countdown()
        _drive(0.36, 0.62)
        time.sleep(3.20)
        _drive(0.0, 0.0)

        log.info("All 4 steps completed successfully!")

    finally:
        # Safely shut down motors on completion or exit
        _drive(0.0, 0.0)
        pi.write(_stby, 0)
        pi.stop()

if __name__ == "__main__":
    main()