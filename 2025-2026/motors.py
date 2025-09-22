from machine import Pin, PWM
from config import RightMotor_m1,RightMotor_m2,RightMotor_pwm,LeftMotor_m1,LeftMotor_m2,LeftMotor_pwm


class motor:
    def __init__(self, m1_Pin, m2_Pin, PWM_Pin):
        
        #motor pins
        self.m1 = Pin(m1_Pin, Pin.OUT)
        self.m2 = Pin(m2_Pin, Pin.OUT)
        
        #pwm
        self.pwm = PWM(Pin(PWM_Pin))
        self.pwm.freq(20000)
        self.last_speed = 0
        self.pwm.duty_u16(int())
        
    def stop_power(self):#for full stop
        self.m1.off()
        self.m2.off()

    def set_speed(self, speed):

            #speed [-1.0, 1.0]
            #Negative -> reverse, Positive -> forward
            
            if speed > 1.0:
                speed = 1.0
            if speed < -1.0:
                speed = -1.0

            duty = int(abs(speed) * 65535)

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
            self.last_speed = speed

RightMotor = motor(RightMotor_m1, RightMotor_m2, RightMotor_pwm)
LeftMotor = motor(LeftMotor_m1, LeftMotor_m2, LeftMotor_pwm)