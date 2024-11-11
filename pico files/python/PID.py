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
        self.dt = 0.1                    #
        
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
        
        pid_output = Pout + Iout + Dout
        
        return pid_output

    def update(self, current_rpm):#function you want to use the most
        self.current_rpm = current_rpm
        
        
        pid_output = self.compute_pid()
        
        return pid_output