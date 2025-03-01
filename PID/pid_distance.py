#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

# Import SunFounder libraries
from picarx import Picarx           # For driving the car (motors & steering)
from picamera2 import Picamera2     # For camera functionality (from vilib)

# -----------------------------
# Configuration & Constants
# -----------------------------
SAMPLETIME = 0.1        # PID loop interval (seconds)
KP = 0.1                # Proportional gain (tune as needed)
KI = 0.01               # Integral gain
KD = 0.05               # Derivative gain

ROTATION_TICKS = 40     # Approximate encoder ticks per full motor rotation
target_ticks = 2 * ROTATION_TICKS  # Target ticks for 2 rotations

# -----------------------------
# Setup GPIO & Encoder Class
# -----------------------------
GPIO.setmode(GPIO.BCM)

class Encoder:
    def __init__(self, pin):
        self._ticks = 0
        self.pin = pin
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Detect both rising and falling edges
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=self._callback)
    
    def _callback(self, channel):
        self._ticks += 1
    
    def reset(self):
        self._ticks = 0
    
    @property
    def ticks(self):
        return self._ticks

# -----------------------------
# Initialize SunFounder Components
# -----------------------------
# Picarx object provides motor and steering control.
px = Picarx()
# Initialize the camera (even if not used in this example)
picam2 = Picamera2()
# (Additional camera configuration can be added here if desired)

# Initialize encoders (assumed connected to GPIO17 and GPIO18)
encoder_left = Encoder(17)
encoder_right = Encoder(18)

# -----------------------------
# PID Variables for Distance Control
# -----------------------------
integral = 0
previous_error = 0

# Use a base speed (PWM value) for forward motion; adjust as needed.
base_speed = 50
current_speed = base_speed

# Start driving forward
px.forward(current_speed)
print("Starting distance-controlled drive...")

# -----------------------------
# PID Loop (Distance Only)
# -----------------------------
while (encoder_left.ticks < target_ticks) and (encoder_right.ticks < target_ticks):
    # Compute average tick count between both wheels
    current_ticks = (encoder_left.ticks + encoder_right.ticks) / 2.0
    # Error is the remaining ticks to reach the target
    error = target_ticks - current_ticks
    
    # PID calculations
    integral += error * SAMPLETIME
    derivative = (error - previous_error) / SAMPLETIME
    output = (KP * error) + (KI * integral) + (KD * derivative)
    previous_error = error
    
    # Adjust speed based on the PID output. As we approach the target, output will decrease.
    # Here, we add the PID output to the base speed. (Depending on tuning, you might subtract instead.)
    new_speed = base_speed + output
    
    # Clamp the new speed between a minimum and maximum value.
    new_speed = max(20, min(100, int(new_speed)))
    current_speed = new_speed
    
    # Set the new speed for both motors
    px.forward(current_speed)
    
    # Debug output to show progress
    print(f"Left ticks: {encoder_left.ticks}, Right ticks: {encoder_right.ticks}, Speed: {current_speed}")
    
    time.sleep(SAMPLETIME)

# Stop the robot when the target distance is reached.
px.forward(0)
print("Target distance reached. Motion complete!")

# Clean up GPIO resources
GPIO.cleanup()
