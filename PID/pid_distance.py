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

# PID Gains (tune these values for your robot)
KP = 1.0
KI = 0.05
KD = 0.2

# Target distance (in centimeters). 2 inches ≈ 5.08 cm, so we use 5 cm as the target.
target_distance = 5.0

# Maximum and minimum speed (PWM value)
max_speed = 100
min_speed = 0

# -----------------------------
# Initialize SunFounder Components
# -----------------------------
# Create the Picarx object for motor & steering control.
px = Picarx()
# Initialize the camera (even if not used directly in this example)
picam2 = Picamera2()

print("Starting distance-controlled drive using the ultrasonic sensor...")

# -----------------------------
# PID Variables for Distance Control
# -----------------------------
integral = 0.0
previous_error = 0.0

while True:
    # Read current distance from the ultrasonic sensor.
    # (Assumes px.get_distance() returns the distance in centimeters.)
    distance = px.get_distance()
    print(f"Measured distance: {distance:.2f} cm")
    
    # If the robot is at or below the target distance, break out of the loop.
    if distance <= target_distance:
        break
    
    # Compute the error: the difference between the measured distance and the target.
    error = distance - target_distance
    
    # PID calculations.
    integral += error * SAMPLETIME
    derivative = (error - previous_error) / SAMPLETIME
    output = (KP * error) + (KI * integral) + (KD * derivative)
    previous_error = error
    
    # Use the PID output to set the speed.
    # When far from the wall, error is high so output (and speed) is high.
    # As the robot approaches, error decreases so the speed is reduced.
    new_speed = int(output)
    
    # Clamp the new speed between min_speed and max_speed.
    if new_speed > max_speed:
        new_speed = max_speed
    if new_speed < min_speed:
        new_speed = min_speed
    
    # Set the new forward speed.
    px.forward(new_speed)
    
    # Debug output.
    print(f"Error: {error:.2f}, PID output (speed command): {new_speed}")
    
    time.sleep(SAMPLETIME)

# Stop the robot once the target distance is reached.
px.forward(0)
print("Target distance reached. Motion complete!")

# Clean up GPIO resources.
GPIO.cleanup()
