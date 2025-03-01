#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys

# HC-SR04 sensor GPIO pin configuration
TRIG_PIN = 23  # Trigger pin
ECHO_PIN = 24  # Echo pin

# Setup GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    """
    Triggers the HC-SR04 ultrasonic sensor and measures the time until echo is received.
    Returns:
        distance (float): Distance in centimeters.
    """
    # Ensure trigger is low initially
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.0002)  # 200 microseconds delay

    # Send a 10 microsecond pulse to trigger the sensor
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # 10 microseconds pulse
    GPIO.output(TRIG_PIN, False)
    
    # Wait for the echo to start
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
    
    # Wait for the echo to end
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
    
    # Calculate pulse duration
    pulse_duration = pulse_end - pulse_start
    
    # Calculate distance (speed of sound is approximately 34300 cm/s)
    distance = (pulse_duration * 34300) / 2
    return distance

if __name__ == "__main__":
    try:
        while True:
            dist = get_distance()
            # Update the same console line using carriage return
            sys.stdout.write("\rDistance: {:.2f} cm".format(dist))
            sys.stdout.flush()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user")
    finally:
        GPIO.cleanup()