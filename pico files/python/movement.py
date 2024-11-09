import _thread
import utime as time
from machine import Pin, PWM, UART

#movement distances
#measured distances for amount of times encoders needs to turn
distanceForward = 0
distanceTurn = 0

#set time goal
GoalTime = 0

#setting motor power pin and turning it on
MotorPower = Pin(0, Pin.OUT)
MotorPower.on()

#takes pins and makes object
class motor:
    def __init__(self, m1_Pin, m2_Pin, c1_Pin, c2_Pin, PWM_Pin):
        
        #motor pins
        self.m1 = Pin(m1_Pin, Pin.OUT)
        self.m2 = Pin(m2_Pin, Pin.OUT)
        
        #encoder pins
        self.c1 = Pin(c1_Pin, Pin.IN)
        self.c2 = Pin(c2_Pin, Pin.IN)
        
        #pwm
        self.pwm = PWM(Pin(PWM_Pin))
        self.pwm.freq(20000)
        self.pwm.duty_u16(int(65536 * 0.5))
        
        #motor tracking
        self.pos = 0
        self.prev_c1_state = self.c1.value()
        self.prev_c2_state = self.c2.value()
    def SpinForward(self):#motor command for forward
        self.m1.on()
        self.m2.off()
    def SpinBackward(self):#motor command for backwards
        self.m1.off()
        self.m2.on()
    def stopPower(self):#makes m1 and m2 off
        self.m1.off()
        self.m2.off()
    def sync(self,target): #makes motor have right pos
        pass

#setting the motors
#name = motor(motorpin1,motorp,in2,sense1,sense2,speedcontrol)
RightMotor = motor(1,2,3,4,5)
LeftMotor = motor(6,7,8,9,10)

#makes motor posistion always useable and makes motor rpm the same
def BothMotorPos():#wrapper function for threads
    RightMotor.update_pos()
    LeftMotor.update_pos()
    RightMotor.sync(LeftMotor)
    time.sleep(0.1)


#finds the speed to get time goal
def getmovementSpeed(target,commands):
    totalCommands = len(commands)
    #assumes 50cm boxes
    rotationsWheels = 0#keeps track for math
    wheelRadius = 0#wheel radius in cm
    width = 0 #width between the wheels
    straightRotations = 50/(wheelRadius*3.14159*2)
    turnRotations = width/(4*wheelRadius)
    startMovement =25/(wheelRadius*3.14159*2)
    for command in commands:
        if command == 0 or 3:
            rotationWheels+=straightRotations
        if command == 1 or 2:
            rotationsWheels+=turnRotations
        else:
            pass
    rotationsWheels+=startMovement
    tpr = target-(totalCommands*(0.1))/rotationsWheels #tpr = time per rotations|in secounds
    return tpr 

#movement commands___________________
#robot forward
def forward(distance,speed):
    MotorPower.on()
    while RightMotor.pos >= (distanceForward*distance):
        RightMotor.PWM_Pin.duty_u16(int(speed))
        LeftMotor.PWM_Pin.duty_16(int(speed))
        RightMotor.SpinForward()
        LeftMotor.SpinForward()
        #stopping it
    LeftMotor.stopPower()
    RightMotor.stopPower()
    MotorPower.off()
    time.sleep(0.1)     
#robot turn right
def right(distance,speed):
    MotorPower.on()
    while RightMotor.pos >= -(distanceTurn*distance):
        RightMotor.PWM_Pin.duty_u16(int(speed))
        LeftMotor.PWM_Pin.duty_16(int(speed))
       