from machine import Pin, PWM
import _thread
import time
import PID as pid

#**************************tuning variables*********************************************
#kP          # proportional constant
#Ki          # integral constant
#Kd          # derivative constant
'''PID Tuning Methodology
Ziegler–Nichols tuning
Start with ki = 0 and kd = 0.Gradually increase kp until you reach the "critical gain" where the system oscillates.
Set kP to half of this critical gain. Gradually add ki and kd to smooth the response.
trial and error adjust kp until you get a fast response without significant overshoot.Increase ki to remove any steady-state error.
Finally, add kd to smooth oscillations if necessary.
Tips for Tuning
Start Small: Begin with low values and increase incrementally to observe the effect.
Stability First: Achieve a stable response with  kP and KD before adding KI
Record Changes: Document different settings and their effects to track improvements or regressions.
Be Patient: It may take several iterations to find the best values.
By carefully adjusting each parameter, you should reach a balance between responsiveness, stability, and accuracy.'''
RightMotor_kp = 0 #rightmotor kp
RightMotor_ki = 0 #right motor Ki
RightMotor_kd = 0 #right motor kd
LeftMotor_kp = 0 #leftmotor kp
LeftMotor_ki = 0 #leftmotor ki
LeftMotor_kd = 0 #leftmotor kd

#motor pins
RightMotor_m1,RightMotor_m2,RightMotor_pwm = 1,2,3
LeftMotor_m1,LeftMotor_m2,LeftMotor_pwm = 4,5,6

#takes pins and makes object
class motor:
    def __init__(self, m1_Pin, m2_Pin, PWM_Pin):
        
        #motor pins
        self.m1 = Pin(m1_Pin, Pin.OUT)
        self.m2 = Pin(m2_Pin, Pin.OUT)
        
        #pwm
        self.pwm = PWM(Pin(PWM_Pin))
        self.pwm.freq(20000)
        self.pwm.duty_u16(int(65536 * 0.5))
        

    def update_pos(self):#updates motor pos
        pass
        #might have to added something based on the read function
    def spin_forward(self):#motor command for forward
        self.m1.on()
        self.m2.off()
    def spin_backward(self):#motor command for backwards
        self.m1.off()
        self.m2.on()
    def stop_power(self):#makes m1 and m2 off
        self.m1.off()
        self.m2.off()
    def sync(self,target): #makes motor have right pos
        pass
'''need to find a function to get current rpms'''

#finds the speed to get time goal in rotations per minuite
def get_movement_speed(target,commands):
    total_commands = len(commands)
    #assumes 50cm boxes
    wheel_rotations = 0#keeps track for math
    wheel_radius = 0#wheel radius in cm
    width = 0 #width between the wheels
    forward_movement = 50/(wheel_radius*3.14159*2)
    first_move = 25/(wheel_radius*3.14159*2)
    turn_movement = width/(4*wheel_radius)
    
    for command in commands:
        if command == 0 or 3:
            wheel_rotations+=forward_movement
        if command == 1 or 2:
            wheel_rotations+=turn_movement
        else:
            pass
    wheel_rotations+=first_move
    tpr = target-(total_commands*(0.1))/wheel_rotations #tpr = time per rotations|in secounds
    return tpr


#setting the motor pins
#name = motor(motorpin1,motorpin2, speedcontrol)
RightMotor = motor(1,4,5)
LeftMotor = motor(6,9,10)

def setting_motors(goal_rpm,rightMotor_current_rpm,leftMotor_current_rpm):#pid function for both motors
    #kP, Ki, Kd, current_rpm, goal_rpm
    RightMotor = motor(LeftMotor_m1,LeftMotor_m2,LeftMotor_pwm)
    LeftMotor = motor(RightMotor_m1,RightMotor_m2,RightMotor_pwm)
    RightMotor_pid = pid.PIDController(RightMotor_kp,RightMotor_ki,RightMotor_kd,rightMotor_current_rpm,goal_rpm)
    LeftMotor_pid = pid.PIDController(LeftMotor_kp,LeftMotor_ki,LeftMotor_kd,leftMotor_current_rpm,goal_rpm)

#movement commands
def forward(distance,rpm):#moves robot forward
    if distance == 1:
        pass
    elif distance == 0.5:
        pass
    else:
        raise ValueError ("distance has to 1 or .5")

def rigth(rpm):#turns robot right
    pass

def left(rpm):#turns robot left
    pass

def oneeighty(rpm):#moves robot backward
    pass

def stop():#stops the motors
    pass