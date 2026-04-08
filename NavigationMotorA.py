from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

# --- Pin Mapping (BCM Numbers) ---
# Physical 32 -> GPIO 12 (PWMA)
# Physical 13 -> GPIO 27 (AIN1)
# Physical 15 -> GPIO 22 (AIN2)
# Physical 16 -> GPIO 23 (STBY)

pwma = PWMOutputDevice(12)
ain1 = DigitalOutputDevice(27)
ain2 = DigitalOutputDevice(22)
stby = DigitalOutputDevice(23)

def motor_forward(speed=0.6):
    stby.on()
    ain1.on()
    ain2.off()
    pwma.value = speed
    print("Moving Forward...")

def motor_backward(speed=0.6):
    stby.on()
    ain1.off()
    ain2.on()
    pwma.value = speed
    print("Moving Backward...")

def motor_stop():
    pwma.value = 0
    # Turning AIN pins off acts as a brake
    ain1.off()
    ain2.off()
    print("Stopped.")

try:
    print("Starting sequence. Press Ctrl+C to exit.")
    while True:
        # 1. Go Forward for 1 second
        motor_forward(0.5)
        sleep(1)

        # 2. Stop for 1 second
        motor_stop()
        sleep(1)

        # 3. Go Backward for 1 second
        motor_backward(0.5)
        sleep(1)

        # 4. Stop for 1 second
        motor_stop()
        sleep(1)

except KeyboardInterrupt:
    print("\nSequence ended by user.")
    motor_stop()
    stby.off() # Disable driver
    

