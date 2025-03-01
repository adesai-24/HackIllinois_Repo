#!/usr/bin/env python3
from gpiozero import DistanceSensor
import time

# Initialize the DistanceSensor:
# Adjust the max_distance (in meters) if needed.
ultrasonic = DistanceSensor(echo=17, trigger=4, max_distance=2)

try:
    while True:
        # Distance in meters (converted to centimeters)
        distance_cm = ultrasonic.distance * 100
        print("Distance: {:.2f} cm".format(distance_cm))
        time.sleep(1)
except KeyboardInterrupt:
    print("\nMeasurement stopped by user")
