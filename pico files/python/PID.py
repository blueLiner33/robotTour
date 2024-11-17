#should return value for pwd pin!
from machine import PWM
class PIDController:
    def __init__(self, kP, Ki, Kd, current_rpm, goal_rpm,pwm_pin):
        #turn these for each motor
        self.kP = kP          # proportional constant
        self.Ki = Ki          # integral constant
        self.Kd = Kd          # derivative constant
        self.pwm_pin = pwm_pin 
        self.current_rpm = current_rpm
        self.goal_rpm = goal_rpm
        self.previous_rpm = current_rpm  
        self.previous_error = 0          
        self.integral = 0               
        self.dt = 0.1
        self.pid_output = 0
        
    def compute_pid(self):
        error = self.goal_rpm - self.current_rpm
        # Pout
        Pout = self.kP * error
        
        # Iout
        self.integral += error * self.dt
        Iout = self.Ki * self.integral
        
        # Dout
        derivative = (error - self.previous_error) / self.dt
        Dout = self.Kd * derivative
        
        self.previous_error = error
        self.previous_rpm = self.current_rpm
        
        self.pid_output = Pout + Iout + Dout

    def update(self, current_rpm):#function you want to use the most have to call to update self.pid_output
        self.current_rpm = current_rpm
        self.compute_pid()
    
    def adjust(self):#makes rpm right
        if self.pid_output < 0:
            current_pwm = self.pwm_pin.duty_u16() 
            adjusted_pwm = current_pwm + int(self.pid_output)
            self.pwm_pin.duty_u16(int(adjusted_pwm))
        elif self.pid_output>0:
            current_pwm = self.pwm_pin.duty_u16() 
            adjusted_pwm = current_pwm - int(self.pid_output)
            self.pwm_pin.duty_u16(int(adjusted_pwm))
        else:
            pass
        