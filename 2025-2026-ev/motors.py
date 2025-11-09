# motors.py
from machine import Pin, PWM
from config import RightMotor_m1, RightMotor_m2, RightMotor_pwm, LeftMotor_m1, LeftMotor_m2, LeftMotor_pwm

MAX_DUTY = 65535

class Motor:
    def __init__(self, m1_Pin, m2_Pin, PWM_Pin):
        self.m1 = Pin(m1_Pin, Pin.OUT)
        self.m2 = Pin(m2_Pin, Pin.OUT)
        self.pwm = PWM(Pin(PWM_Pin))
        # PWM frequency (tune if necessary)
        self.pwm.freq(20000)
        self._last_speed = 0.0
        # ensure motors are stopped
        self.m1.off()
        self.m2.off()
        self.pwm.duty_u16(0)

    def set_speed(self, speed):
        """
        Set motor speed:
        speed: float in range [-1.0, 1.0]
        Negative -> reverse, Positive -> forward
        """
        if speed > 1.0:
            speed = 1.0
        if speed < -1.0:
            speed = -1.0

        duty = int(abs(speed) * MAX_DUTY)
        # direction depending on sign
        if speed > 0:
            self.m1.on()
            self.m2.off()
        elif speed < 0:
            self.m1.off()
            self.m2.on()
        else:
            self.m1.off()
            self.m2.off()
            duty = 0

        self.pwm.duty_u16(duty)
        self._last_speed = speed

    def stop(self):
        self.set_speed(0.0)

# create instances
RightMotor = Motor(RightMotor_m1, RightMotor_m2, RightMotor_pwm)
LeftMotor = Motor(LeftMotor_m1, LeftMotor_m2, LeftMotor_pwm)
