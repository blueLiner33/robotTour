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
        self.pwm.duty_u16(int(65536 * 0.5))
        
    def spin_forward(self):#motor command for forward
        self.m1.on()
        self.m2.off()
        
    def spin_backward(self):#motor command for backwards
        self.m1.off()
        self.m2.on()
        
    def stop_power(self):#makes m1 and m2 off
        self.m1.off()
        self.m2.off()

class robot_movements:
    def __init__(self, rightMotor, leftMotor):
        self.rightMotor = rightMotor
        self.leftMotor = leftMotor

    def move_forward(self):
        self.rightMotor.spin_forward()
        self.leftMotor.spin_forward()
        
    def move_backward(self):
        self.rightMotor.spin_backward()
        self.leftMotor.spin_backward()

    def move_right(self):
        self.rightMotor.spin_backward()
        self.leftMotor.spin_forward()
    def move_left(self):
        self.rightMotor.spin_forward()
        self.leftMotor.spin_backward()


RightMotor = motor(RightMotor_m1, RightMotor_m2, RightMotor_pwm)
LeftMotor = motor(LeftMotor_m1, LeftMotor_m2, LeftMotor_pwm)

car  = robot_movements(RightMotor, LeftMotor)