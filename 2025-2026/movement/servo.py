from config.config import servo_pin
from machine import PWM, Pin
import time

class servo:
    def __init__(self, servo_pin):
        self.servo_pin = PWM(Pin(servo_pin))
        self.servo_pin.freq(50)

    def set_angle(self, angle): 
        max_duty = 8192  
        min_duty = 1638   
        duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
        self.servo_pin.duty_u16(duty)

    def open (self):
        for angle in range(0, 90, 5): 
            self.set_angle(angle)
            time.sleep(0.05)  

    def close (self):
        for angle in range(90, 0, 5):  
            self.set_angle(angle)
            time.sleep(0.05)  

grab_latch = servo(servo_pin)