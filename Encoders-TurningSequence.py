"""
Encoders-TurningSequence.py

Integrates EncoderReader with open-loop and closed-loop motor turning sequences.
Allows driving fixed distances/angles based on encoder count targets instead of fixed time delays.
"""

import math
import time
import logging
from dataclasses import dataclass
import pigpio
from System import System

# =============================================================================
# Logging Setup
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("pipeline")


# =============================================================================
# Telemetry Data Containers
# =============================================================================
@dataclass
class EncoderFrame:
    """Encoder pulse counts and calculated speeds for one frame window."""
    left_count: int = 0
    right_count: int = 0
    left_cps: float = 0.0   # Counts per second
    right_cps: float = 0.0  # Counts per second


# =============================================================================
# N20 Encoder Reader Class (pigpio Hardware Interrupts)
# =============================================================================
class EncoderReader:
    """
    Reads quadrature encoders on Left (GPIO 16/19) and Right (GPIO 21/20) motors.
    """
    LEFT_C1  = 16
    LEFT_C2  = 19
    RIGHT_C1 = 21
    RIGHT_C2 = 20

    def __init__(self, pi: pigpio.pi) -> None:
        self._pi = pi
        if not self._pi.connected:
            raise RuntimeError("pigpio daemon not reachable. Run: sudo pigpiod")

        self._left_pos = 0
        self._right_pos = 0
        self._last_time = time.perf_counter()

        self._left_c1_state = 0
        self._left_c2_state = 0
        self._right_c1_state = 0
        self._right_c2_state = 0

        # Configure GPIO modes and pull-ups
        for pin in [self.LEFT_C1, self.LEFT_C2, self.RIGHT_C1, self.RIGHT_C2]:
            self._pi.set_mode(pin, pigpio.INPUT)
            self._pi.set_pull_up_down(pin, pigpio.PUD_UP)

        # Set up callbacks for quadrature decoding
        self._cb_l1 = self._pi.callback(self.LEFT_C1, pigpio.EITHER_EDGE, self._left_cb)
        self._cb_l2 = self._pi.callback(self.LEFT_C2, pigpio.EITHER_EDGE, self._left_cb)
        self._cb_r1 = self._pi.callback(self.RIGHT_C1, pigpio.EITHER_EDGE, self._right_cb)
        self._cb_r2 = self._pi.callback(self.RIGHT_C2, pigpio.EITHER_EDGE, self._right_cb)

    def _left_cb(self, gpio: int, level: int, tick: int) -> None:
        if gpio == self.LEFT_C1:
            self._left_c1_state = level
        elif gpio == self.LEFT_C2:
            self._left_c2_state = level

        if gpio == self.LEFT_C1 and level == 1:
            if self._left_c2_state == 0:
                self._left_pos += 1
            else:
                self._left_pos -= 1

    def _right_cb(self, gpio: int, level: int, tick: int) -> None:
        if gpio == self.RIGHT_C1:
            self._right_c1_state = level
        elif gpio == self.RIGHT_C2:
            self._right_c2_state = level

        if gpio == self.RIGHT_C1 and level == 1:
            if self._right_c2_state == 0:
                self._right_pos += 1
            else:
                self._right_pos -= 1

    def snapshot(self) -> EncoderFrame:
        """Atomically read current counts and compute counts per second."""
        now = time.perf_counter()
        dt = now - self._last_time
        self._last_time = now

        l_count = self._left_pos
        r_count = self._right_pos

        l_cps = (l_count / dt) if dt > 0 else 0.0
        r_cps = (r_count / dt) if dt > 0 else 0.0

        return EncoderFrame(
            left_count=l_count,
            right_count=r_count,
            left_cps=l_cps,
            right_cps=r_cps,
        )

    def reset(self) -> None:
        """Reset internal encoder count offsets to zero."""
        self._left_pos = 0
        self._right_pos = 0

    def cancel(self) -> None:
        """Clean up pigpio callbacks."""
        self._cb_l1.cancel()
        self._cb_l2.cancel()
        self._cb_r1.cancel()
        self._cb_r2.cancel()


# =============================================================================
# Motor Driver Helpers
# =============================================================================
_ain1 = 24
_ain2 = 25
_pwma = 13

_bin1 = 27
_bin2 = 22
_pwmb = 12
_stby = 23


def init_motors(pi: pigpio.pi) -> None:
    """Initialize GPIO pins for the TB6612 motor driver."""
    pi.set_mode(_ain1, pigpio.OUTPUT)
    pi.set_mode(_ain2, pigpio.OUTPUT)
    pi.set_mode(_bin1, pigpio.OUTPUT)
    pi.set_mode(_bin2, pigpio.OUTPUT)
    pi.set_mode(_stby, pigpio.OUTPUT)


def drive(pi: pigpio.pi, left_speed: float, right_speed: float) -> None:
    """
    Drive the robot with specified left and right motor speeds (-1.0 to 1.0).
    """
    pi.write(_stby, 1)

    # Left Motor
    spd_l = int(max(0.0, min(1.0, abs(left_speed))) * 1000000)
    pi.hardware_PWM(_pwma, 1000, spd_l)
    pi.write(_ain1, 1 if left_speed < 0 else 0)
    pi.write(_ain2, 1 if left_speed > 0 else 0)

    # Right Motor
    spd_r = int(max(0.0, min(1.0, abs(right_speed))) * 1000000)
    pi.hardware_PWM(_pwmb, 1000, spd_r)
    pi.write(_bin1, 1 if right_speed > 0 else 0)
    pi.write(_bin2, 1 if right_speed < 0 else 0)


# =============================================================================
# Controlled Movement Helper Functions
# =============================================================================
def drive_for_duration(
    pi: pigpio.pi,
    encoders: EncoderReader,
    left_speed: float,
    right_speed: float,
    duration: float,
) -> None:
    """
    Drives at requested speeds while logging encoder tick telemetry.
    """
    encoders.reset()
    start_time = time.perf_counter()
    
    drive(pi, left_speed, right_speed)

    while (time.perf_counter() - start_time) < duration:
        frame = encoders.snapshot()
        log.info(
            f"Driving... Elapsed: {time.perf_counter() - start_time:.2f}s | "
            f"L: {frame.left_count} ticks ({frame.left_cps:.1f} cps) | "
            f"R: {frame.right_count} ticks ({frame.right_cps:.1f} cps)"
        )
        time.sleep(0.05)

    drive(pi, 0.0, 0.0)


def drive_target_ticks(
    pi: pigpio.pi,
    encoders: EncoderReader,
    left_speed: float,
    right_speed: float,
    target_left_ticks: int | None = None,
    target_right_ticks: int | None = None,
    timeout: float = 5.0,
) -> None:
    """
    Drives until reaching specified target encoder counts or timeout.
    Set target_left_ticks or target_right_ticks to control stop condition.
    """
    encoders.reset()
    start_time = time.perf_counter()

    drive(pi, left_speed, right_speed)

    while (time.perf_counter() - start_time) < timeout:
        frame = encoders.snapshot()

        l_reached = target_left_ticks is None or abs(frame.left_count) >= abs(target_left_ticks)
        r_reached = target_right_ticks is None or abs(frame.right_count) >= abs(target_right_ticks)

        log.info(
            f"Target Drive | L: {frame.left_count}/{target_left_ticks or 'N/A'} | "
            f"R: {frame.right_count}/{target_right_ticks or 'N/A'}"
        )

        if l_reached and r_reached:
            log.info("Reached target encoder count.")
            break

        time.sleep(0.02)

    drive(pi, 0.0, 0.0)


# =============================================================================
# Main Sequence
# =============================================================================
def main() -> None:
    log.info("Starting Navilott Pipeline with Encoders...")

    pi = pigpio.pi()
    if not pi.connected:
        log.error("Failed to connect to pigpio daemon. Run 'sudo pigpiod' first.")
        return

    init_motors(pi)
    encoders = EncoderReader(pi)
    system = System()

    try:
        # Step 1: Approach stop line (22cm approach window)
        system.wait_for_start()
        system.run_countdown()
        log.info("Step 1: Approaching stop line...")
        drive_for_duration(pi, encoders, left_speed=0.45, right_speed=0.45, duration=2.10)

        # Step 2: Cross stop line threshold
        system.wait_for_start()
        system.run_countdown()
        log.info("Step 2: Crossing stop line...")
        drive_for_duration(pi, encoders, left_speed=0.45, right_speed=0.45, duration=1.25)

        # Step 3: Right turn sequence (90 degree turn)
        system.wait_for_start()
        system.run_countdown()
        log.info("Step 3: Executing Right Turn...")
        drive_for_duration(pi, encoders, left_speed=0.45, right_speed=0.0, duration=1.62)

        # Step 4: Left turn sequence
        system.wait_for_start()
        system.run_countdown()
        log.info("Step 4: Executing Left Turn...")
        drive_for_duration(pi, encoders, left_speed=0.36, right_speed=0.62, duration=2.75)

        log.info("All 4 steps completed successfully!")

    finally:
        drive(pi, 0.0, 0.0)
        pi.write(_stby, 0)
        encoders.cancel()
        pi.stop()
        log.info("Cleanup complete.")


if __name__ == "__main__":
    main()