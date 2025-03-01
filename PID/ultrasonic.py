#!/usr/bin/env python3
import time
from picarx import Picarx

if __name__ == "__main__":
    px = Picarx()
    time.sleep(2)  # Allow sensor to settle
    print("Starting ultrasonic sensor test. Press Ctrl+C to stop.")
    try:
        while True:
            distance = px.get_distance()
            print("Distance: {:.2f}".format(distance))
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
