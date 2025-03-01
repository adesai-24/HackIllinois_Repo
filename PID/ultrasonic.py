#!/usr/bin/env python3
import time
from picarx import Picarx

if __name__ == "__main__":
    # Initialize the PiCar-X (this resets the MCU and sets up all sensors/servos)
    px = Picarx()
    
    print("Starting ultrasonic sensor test. Press Ctrl+C to stop.")
    
    try:
        while True:
            # Use the built-in ultrasonic sensor method to get the current distance.
            # (The returned unit depends on the library—commonly centimeters.)
            distance = px.get_distance()
            print("Distance: {:.2f}".format(distance))
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
