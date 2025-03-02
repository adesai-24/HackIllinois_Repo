from gpiozero import Robot, DigitalInputDevice
from time import sleep
import smbus2 as smbus  # For gyroscope readings

# Constants
SAMPLETIME = 0.1  # Sample time in seconds
KP = 0.05  # Proportional gain
KI = 0.01  # Integral gain
KD = 0.02  # Derivative gain
TARGET_ANGLE = 0  # Target gyroscope angle for straight movement-needs to be altered for a specific angle
ROTATION_TICKS = 40  # Approximate encoder ticks for one full motor rotation

# PID variables
integral = 0
previous_error = 0

# Encoder class
class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        encoder = DigitalInputDevice(pin)
        encoder.when_activated = self._increment
        encoder.when_deactivated = self._increment
    
    def reset(self):
        self._value = 0
    
    def _increment(self):
        self._value += 1
    
    @property
    def value(self):
        return self._value

# Gyroscope class (example using I2C interface)
class Gyroscope:
    def __init__(self, address=0x68):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake up MPU6050
    
    def get_angle(self):
        high = self.bus.read_byte_data(self.address, 0x43)
        low = self.bus.read_byte_data(self.address, 0x44)
        angle = (high << 8) + low
        if angle > 32768:
            angle -= 65536
        return angle / 131.0  # Convert raw data to degrees per second

# Initialize robot, encoders, and gyroscope
r = Robot((10,9), (8,7))
e1 = Encoder(17)
e2 = Encoder(18)
gyro = Gyroscope()

# Start motors at base speed
base_speed = 0.5
m1_speed = base_speed
m2_speed = base_speed
r.value = (m1_speed, m2_speed)

target_ticks = 2 * ROTATION_TICKS  # Two full motor rotations

# Move forward with PID correction for straight travel
while e1.value < target_ticks and e2.value < target_ticks:
    # Read gyroscope angle
    angle_error = TARGET_ANGLE - gyro.get_angle()
    
    # PID calculations
    integral += angle_error * SAMPLETIME
    derivative = (angle_error - previous_error) / SAMPLETIME
    correction = (KP * angle_error) + (KI * integral) + (KD * derivative)
    previous_error = angle_error
    
    # Adjust motor speeds
    m1_speed = base_speed - correction
    m2_speed = base_speed + correction
    
    # Clamp motor speeds
    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)
    
    # Apply motor speed corrections
    r.value = (m1_speed, m2_speed)
    
    # Debug output
    print(f"Angle Error: {angle_error:.2f}, Correction: {correction:.2f}")
    print(f"Motor Speeds -> m1: {m1_speed:.2f}, m2: {m2_speed:.2f}")
    
    sleep(SAMPLETIME)

# Stop motors
r.value = (0, 0)
print("Motion complete")