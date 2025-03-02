from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
import time

# Use the pigpio pin factory for more accurate timing (make sure pigpiod is running)
factory = PiGPIOFactory()

# Initialize the ultrasonic sensor: trigger on GPIO17, echo on GPIO18.
sensor = DistanceSensor(echo=18, trigger=17, max_distance=2, pin_factory=factory)

while True:
    # sensor.distance returns the distance in meters; convert to cm for display
    print("Distance: {:.2f} cm".format(sensor.distance * 100))
    time.sleep(1)
