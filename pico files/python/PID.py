#should return value for pwd pin!
class PIDController:
    def __init__(self, kP, Ki, Kd, current_rpm, goal_rpm):
        #turn these for each motor
        self.kP = kP          # proportional constant
        self.Ki = Ki          # integral constant
        self.Kd = Kd          # derivative constant
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
        self.pwm_pin = 50#need to fix this part
        