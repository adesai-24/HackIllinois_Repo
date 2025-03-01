import time
from gpiozero import Robot, DigitalInputDevice
import smbus2 as smbus  # For gyroscope readings

# --- Constants ---
SAMPLETIME = 0.1          
KP = 0.05                 
KI = 0.01                 
KD = 0.02                 
TARGET_ANGLE = 0         
ROTATION_TICKS = 40       
target_ticks = 2 * ROTATION_TICKS  


integral = 0
previous_error = 0


class Encoder:
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


class Gyroscope:
    def __init__(self, address=0x68):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.bus.write_byte_data(self.address, 0x6B, 0)  
    
    def get_angle(self):
        high = self.bus.read_byte_data(self.address, 0x43)
        low = self.bus.read_byte_data(self.address, 0x44)
        angle = (high << 8) + low
        if angle > 32768:
            angle -= 65536
        return angle / 131.0 


r = Robot((4,),(5,))
e1 = Encoder(17)
e2 = Encoder(18)
gyro = Gyroscope()


base_speed = 0.5
m1_speed = base_speed
m2_speed = base_speed
r.value = (m1_speed, m2_speed)

print("Starting PID distance drive...")


while e1.value < target_ticks and e2.value < target_ticks:

    angle_error = TARGET_ANGLE - gyro.get_angle()
    

    integral += angle_error * SAMPLETIME
    derivative = (angle_error - previous_error) / SAMPLETIME
    correction = (KP * angle_error) + (KI * integral) + (KD * derivative)
    previous_error = angle_error
    

    m1_speed = base_speed - correction
    m2_speed = base_speed + correction
    

    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)
    

    r.value = (m1_speed, m2_speed)
    
    print(f"Encoder1: {e1.value}, Encoder2: {e2.value}")
    print(f"Angle error: {angle_error:.2f}, Correction: {correction:.2f}")
    print(f"Motor speeds -> m1: {m1_speed:.2f}, m2: {m2_speed:.2f}")
    
    time.sleep(SAMPLETIME)

r.value = (0, 0)
print("Motion complete!")
