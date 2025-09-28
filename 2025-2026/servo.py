from machine import PWM, Pin
import time
from config import servo_pin

class Servo:
    def __init__(self, servo_pin, freq=50, min_ms=0.5, max_ms=2.4):
        self.pwm = PWM(Pin(servo_pin))
        self.pwm.freq(freq)
        self.min_ms = min_ms
        self.max_ms = max_ms
        self.period_ms = 1000 / freq 

    def _ms_to_duty_u16(self, ms):
        return int((ms / self.period_ms) * 65535)

    def set_angle(self, angle):
        if angle < 0: angle = 0
        if angle > 180: angle = 180
        ms = self.min_ms + (angle / 180.0) * (self.max_ms - self.min_ms)
        self.pwm.duty_u16(self._ms_to_duty_u16(ms))

    def open(self):
        self.set_angle(90)

    def close(self):
        self.set_angle(0)

grap_latch = Servo(servo_pin)
