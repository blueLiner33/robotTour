class PIDController:
    de __init__(self,kP, Ki, Kd):
        self.kP = kP
        self.Ki = Ki 
        self.Kd = Kd 

        self.previous_error = 0
        self.integral = 0 