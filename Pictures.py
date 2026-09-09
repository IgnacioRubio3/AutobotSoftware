import os
import time
import logging
import cv2
from MikeBigStuff import System

# =============================================================================
# Setup & Configuration
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("camera_capture")

# Set up the download directory path dynamically across different operating systems
DOWNLOADS_DIR = os.path.expanduser("~/Downloads")
os.makedirs(DOWNLOADS_DIR, exist_ok=True)

# GStreamer pipeline for IMX219 camera
pipeline = (
    "libcamerasrc ! "
    "video/x-raw,colorimetry=bt709,width=480,height=360,framerate=20/1 ! "
    "videoconvert ! "
    "videoflip method=rotate-180 ! "
    "appsink drop=true max-buffers=1 sync=false"
)

# =============================================================================
# Main Execution Loop
# =============================================================================
def main() -> None:
    # Initialize the camera stream
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        log.error("Failed to open camera pipeline")
        raise SystemExit(1)

    # Initialize the button & display controller
    sys_ctrl = System()
    img_counter = 0

    log.info("System ready. Press the button on GPIO 17 to capture an image.")

    try:
        while True:
            # 1. Wait for user button press (blocking until pressed and released)
            sys_ctrl.wait_for_start()

            # 2. Flush old buffer frames to get a fresh capture
            for _ in range(3):
                cap.grab()

            ret, frame = cap.read()
            if not ret:
                log.warning("Failed to grab image from camera pipeline.")
                continue

            # 3. Generate a timestamped file path in the Downloads folder
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}_{img_counter:03d}.jpg"
            filepath = os.path.join(DOWNLOADS_DIR, filename)

            # 4. Save image
            if cv2.imwrite(filepath, frame):
                img_counter += 1
                log.info("Captured image #%d: %s", img_counter, filepath)
            else:
                log.error("Failed to write image to disk: %s", filepath)

    except KeyboardInterrupt:
        log.info("\nProgram stopped by user.")

    finally:
        # Clean up resources
        cap.release()
        sys_ctrl.cleanup()
        log.info("Camera and GPIO resources released. Total captured: %d", img_counter)

if __name__ == "__main__":
    main()