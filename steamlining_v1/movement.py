from machine import Pin, PWM, UART
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
RightMotor_kp = 1 #rightmotor kp
RightMotor_ki = 0.01 #right motor Ki
RightMotor_kd = 0.01 #right motor kd
LeftMotor_kp = 1 #leftmotor kp
LeftMotor_ki = 0.01 #leftmotor ki
LeftMotor_kd = 0.01 #leftmotor kd

#motor pins
RightMotor_m1,RightMotor_m2,RightMotor_pwm = 14,13,15
LeftMotor_m1,LeftMotor_m2,LeftMotor_pwm = 19,20,16

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
        
    def spin_forward(self):#motor command for forward
        self.m1.on()
        self.m2.off()
    def spin_backward(self):#motor command for backwards
        self.m1.off()
        self.m2.on()
    def stop_power(self):#makes m1 and m2 off
        self.m1.off()
        self.m2.off()


rpm = 25
#creating motors
RightMotor = motor(RightMotor_m1, RightMotor_m2, RightMotor_pwm)
LeftMotor = motor(LeftMotor_m1, LeftMotor_m2, LeftMotor_pwm)

RightMotor_pid = pid.PIDController(RightMotor_kp, RightMotor_ki, RightMotor_kd, 0, rpm, RightMotor.pwm)
LeftMotor_pid = pid.PIDController(LeftMotor_kp, LeftMotor_ki, LeftMotor_kd, 0, rpm, LeftMotor.pwm)




distance_traveled = 0  # Distance traveled in cm
velocity = 0  # Velocity in cm/s


def distance_moved(acceleration):#updates distance
    global distance_traveled, velocity
    time_interval = 0.011
    if acceleration>=0:
        acceleration_cm = acceleration * 100
        velocity += acceleration_cm * time_interval  # v = v + a * dt
        distance_increment = velocity * time_interval  # s = s + v * dt
        distance_traveled += distance_increment
        #return distance_traveled
    else:
        pass
    
def get_distance_traveled():# for testing
    return distance_traveled

#movement commands 
def stop():#stops the motors
    RightMotor.stop_power()
    LeftMotor.stop_power()
    

def start_forward(data):#move forward starting(has to be half as much)
    global distance_traveled
    if data <= 25:
        RightMotor.spin_forward()
        LeftMotor.spin_forward()
        return False
    else:
        stop()
        return True

def forward(data):#moves robot forward
    global distance_traveled
    if data <= 50:
        RightMotor.spin_forward()
        LeftMotor.spin_forward()
        return False
    else:
        stop()
        return True

def right(data):#turns robot right
    if data <= 90:
        RightMotor.spin_backward()
        LeftMotor.spin_forward()
        return False
    else:
        stop()
        return True

def left(data):#turns robot left
    if abs(data) <= 90:
        RightMotor.spin_forward()
        LeftMotor.spin_backward()
        return False
    else:
        stop()
        return True

def oneeighty(data):#moves robot oneeighty
    if abs(data) <= -180:
        RightMotor.spin_forward()
        LeftMotor.spin_backward()
    else:
        stop()