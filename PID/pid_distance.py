#!/usr/bin/env python3
import time
import smbus2 as smbus
import RPi.GPIO as GPIO

# Import SunFounder libraries
from picarx import Picarx           # For driving the car (motors + steering servo)
from picamera2 import Picamera2     # For camera functionality (from vilib)

# -----------------------------
# Configuration and Constants
# -----------------------------
SAMPLETIME = 0.1       # seconds between PID updates
KP = 0.05              # Proportional gain (tune these for your setup)
KI = 0.01              # Integral gain
KD = 0.02              # Derivative gain

TARGET_ANGLE = 0       # desired heading (in degrees, from the gyro)
ROTATION_TICKS = 40    # approximate encoder ticks per full motor rotation
target_ticks = 2 * ROTATION_TICKS  # we want the robot to move 2 rotations

# -----------------------------
# Setup for GPIO (Encoder)
# -----------------------------
GPIO.setmode(GPIO.BCM)

class Encoder:
    def __init__(self, pin):
        self.pin = pin
        self._ticks = 0
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin, GPIO.BOTH, callback=self._callback)
    
    def _callback(self, channel):
        self._ticks += 1
    
    def reset(self):
        self._ticks = 0
    
    @property
    def ticks(self):
        return self._ticks

# -----------------------------
# Gyroscope (MPU6050) Interface
# -----------------------------
class Gyroscope:
    def __init__(self, address=0x68):
        self.bus = smbus.SMBus(1)
        self.address = address
        # Wake up MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)
    
    def get_angle(self):
        # Read high and low bytes from the gyroscope registers
        high = self.bus.read_byte_data(self.address, 0x43)
        low = self.bus.read_byte_data(self.address, 0x44)
        angle = (high << 8) + low
        if angle > 32768:
            angle -= 65536
        return angle / 131.0  # convert raw reading to degrees

# -----------------------------
# Initialize SunFounder Components
# -----------------------------
# Picarx object provides motor and steering control.
px = Picarx()

# Initialize the camera (vilib's picamera2)
picam2 = Picamera2()
# (Additional camera configuration can be done here if desired)

# Initialize encoders.
# (Assume left encoder is on GPIO17 and right encoder on GPIO18.)
encoder_left = Encoder(17)
encoder_right = Encoder(18)

# Initialize the gyroscope.
gyro = Gyroscope()

# -----------------------------
# PID Variables
# -----------------------------
integral = 0
previous_error = 0

# -----------------------------
# Start Driving
# -----------------------------
# Set a base forward speed.
# (For PiCar‑X, forward(speed) typically sets a PWM value; adjust as needed.)
base_speed = 50  
px.forward(base_speed)  # start moving forward

print("Starting PID-controlled distance drive...")

while encoder_left.ticks < target_ticks and encoder_right.ticks < target_ticks:
    # Get current heading error from the gyro
    angle_error = TARGET_ANGLE - gyro.get_angle()
    
    # Compute PID terms
    integral += angle_error * SAMPLETIME
    derivative = (angle_error - previous_error) / SAMPLETIME
    correction = (KP * angle_error) + (KI * integral) + (KD * derivative)
    previous_error = angle_error
    
    # For PiCar‑X, steering is controlled by a servo.
    # Assume a neutral (straight) steering angle is 90°.
    # Adjust the steering angle by the PID correction.
    steering_angle = 90 + correction
    # Clamp the steering angle to safe limits (e.g., 60° to 120°)
    steering_angle = max(min(steering_angle, 120), 60)
    px.set_dir_servo_angle(steering_angle)
    
    # Debug output
    print(f"Left ticks: {encoder_left.ticks}, Right ticks: {encoder_right.ticks}")
    print(f"Angle error: {angle_error:.2f}, Correction: {correction:.2f}")
    print(f"Steering angle set to: {steering_angle:.2f}")
    
    time.sleep(SAMPLETIME)

# Once the target distance is reached, stop the robot.
px.forward(0)
print("Motion complete!")

# Clean up GPIO resources
GPIO.cleanup()
