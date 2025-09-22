import time
from config import turn_kd,turn_ki,turn_kp,forward_kd,forward_ki,forward_kp

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0
        self.error_sum = 0
        self.prev_error = 0

    def update(self, process_variable, dt):
        error = self.setpoint - process_variable
        self.error_sum += error * dt
        d_error = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.error_sum + self.Kd * d_error
        self.prev_error = error
        return output


class CAR:
    def __init__(self, turn_kd, turn_ki, turn_kp, forward_kd, forward_ki, forward_kp):#not sure what else to add here. 
        