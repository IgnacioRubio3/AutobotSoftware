"""
telemetry.py

Combined IMU background accumulator and Dual N20 Encoder tracking 
for the Navilott pipeline using pigpio and Adafruit MPU-6050.
"""

import math
import time
import threading
import logging
from dataclasses import dataclass

import board
import busio
import adafruit_mpu6050
import pigpio

log = logging.getLogger(__name__)


# =============================================================================
# Output Containers
# =============================================================================

@dataclass
class IMUFrame:
    """
    Aggregated IMU values for one pipeline frame window.
    """
    mean_yaw_rate_dps  : float | None = None
    peak_lateral_accel : float | None = None
    sample_count       : int          = 0
    valid              : bool         = False


@dataclass
class EncoderFrame:
    """
    Encoder pulse counts and calculated speeds for one pipeline frame window.
    """
    left_count  : int   = 0
    right_count : int   = 0
    left_cps    : float = 0.0  # Counts per second
    right_cps   : float = 0.0  # Counts per second


# =============================================================================
# N20 Encoder Reader (Quadrature Decoding via pigpio)
# =============================================================================

class EncoderReader:
    """
    Reads quadrature encoders on Left (GPIO 16/19) and Right (GPIO 21/20) motors.
    Uses pigpio hardware interrupts for reliable pulse counting.
    """

    # BCM GPIO Assignments matching physical pins:
    # Left: Pin 36 (C1) = BCM 16, Pin 35 (C2) = BCM 19
    # Right: Pin 40 (C1) = BCM 21, Pin 38 (C2) = BCM 20
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

        # Configure GPIO modes and internal pull-ups
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

        # Quadrature step logic
        if (gpio == self.LEFT_C1 and level == 1):
            if self._left_c2_state == 0:
                self._left_pos += 1
            else:
                self._left_pos -= 1

    def _right_cb(self, gpio: int, level: int, tick: int) -> None:
        if gpio == self.RIGHT_C1:
            self._right_c1_state = level
        elif gpio == self.RIGHT_C2:
            self._right_c2_state = level

        if (gpio == self.RIGHT_C1 and level == 1):
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

    def cancel(self) -> None:
        """Clean up pigpio callbacks."""
        self._cb_l1.cancel()
        self._cb_l2.cancel()
        self._cb_r1.cancel()
        self._cb_r2.cancel()


# =============================================================================
# IMU Reader (Unmodified Core)
# =============================================================================

class IMUReader:
    """
    Initializes the MPU-6050 and manages a background sampling thread.
    """

    def __init__(
        self,
        address   : int   = 0x68,
        rate_hz   : float = 100.0,
    ) -> None:
        i2c       = busio.I2C(board.SCL, board.SDA)
        self._mpu = adafruit_mpu6050.MPU6050(i2c, address=address)

        self._rate_hz  = rate_hz
        self._lock     = threading.Lock()
        self._stop_evt = threading.Event()
        self._yaw_buf  : list[float] = []
        self._accel_buf: list[float] = []
        self._thread   : threading.Thread | None = None

    def start(self) -> None:
        """Start the background sampling thread."""
        self._thread = threading.Thread(
            target  = self._worker,
            daemon  = True,
            name    = "imu-accumulator",
        )
        self._thread.start()
        log.info("IMUReader started at %.0f Hz on 0x%02X", self._rate_hz, 0x68)

    def stop(self, timeout: float = 0.5) -> None:
        """Signal the worker to stop and wait for it to exit."""
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        log.info("IMUReader stopped.")

    def snapshot(self) -> IMUFrame:
        """Atomically drain the accumulation buffers."""
        with self._lock:
            yaw_buf   = self._yaw_buf.copy()
            accel_buf = self._accel_buf.copy()
            self._yaw_buf.clear()
            self._accel_buf.clear()

        n = len(yaw_buf)
        if n == 0:
            return IMUFrame()

        return IMUFrame(
            mean_yaw_rate_dps  = sum(yaw_buf) / n,
            peak_lateral_accel = max(accel_buf, key=abs),
            sample_count       = n,
            valid              = True,
        )

    def _worker(self) -> None:
        interval = 1.0 / self._rate_hz
        while not self._stop_evt.is_set():
            t0 = time.perf_counter()
            try:
                gx, gy, gz = self._mpu.gyro          # rad/s
                ax, ay, az = self._mpu.acceleration   # m/s²
                yaw_dps    = gz * (180.0 / math.pi)
                with self._lock:
                    self._yaw_buf.append(yaw_dps)
                    self._accel_buf.append(ay)
            except Exception as exc:
                log.debug("IMU read error (skipped): %s", exc)

            sleep_t = interval - (time.perf_counter() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)


# =============================================================================
# Unified Pipeline Telemetry Runner
# =============================================================================

def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    # Connect to pigpio daemon
    pi = pigpio.pi()
    if not pi.connected:
        log.error("Failed to connect to pigpio daemon. Run 'sudo pigpiod' first.")
        return

    encoder_reader = EncoderReader(pi)
    imu_reader = IMUReader()
    imu_reader.start()

    log.info("Starting combined Telemetry Stream. Press Ctrl+C to stop.\n")

    # Header display
    header = f"| {'Time (s)':^8} | {'Yaw (deg/s)':^12} | {'Lat Acc (m/s²)':^14} | {'L Count':^8} | {'R Count':^8} | {'L (cnt/s)':^10} | {'R (cnt/s)':^10} |"
    divider = "-" * len(header)
    print(divider)
    print(header)
    print(divider)

    start_time = time.perf_counter()

    try:
        while True:
            time.sleep(0.1)  # 10 Hz Telemetry Loop
            
            elapsed = time.perf_counter() - start_time
            imu_data = imu_reader.snapshot()
            enc_data = encoder_reader.snapshot()

            yaw_str = f"{imu_data.mean_yaw_rate_dps:12.2f}" if imu_data.valid and imu_data.mean_yaw_rate_dps is not None else f"{'N/A':^12}"
            acc_str = f"{imu_data.peak_lateral_accel:14.2f}" if imu_data.valid and imu_data.peak_lateral_accel is not None else f"{'N/A':^14}"

            row = (
                f"| {elapsed:8.1f} | "
                f"{yaw_str} | "
                f"{acc_str} | "
                f"{enc_data.left_count:8d} | "
                f"{enc_data.right_count:8d} | "
                f"{enc_data.left_cps:10.1f} | "
                f"{enc_data.right_cps:10.1f} |"
            )
            print(row)

    except KeyboardInterrupt:
        print(divider)
        log.info("Stopping telemetry stream...")
    finally:
        imu_reader.stop()
        encoder_reader.cancel()
        pi.stop()
        log.info("Cleanup complete.")


if __name__ == "__main__":
    main()