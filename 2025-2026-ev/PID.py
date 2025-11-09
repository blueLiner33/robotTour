# PID.py
import time

class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, windup_guard=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_guard = windup_guard

        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def reset(self):
        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def update(self, error, now=None):
        # error: current error (e.g., angle error in radians)
        # returns control output
        if now is None:
            now = time.ticks_ms() / 1000.0

        if self.last_time is None:
            dt = 0.0
        else:
            dt = now - self.last_time
            if dt <= 0:
                dt = 1e-6

        # integral with windup guard
        self.integral += error * dt
        if self.integral > self.windup_guard:
            self.integral = self.windup_guard
        elif self.integral < -self.windup_guard:
            self.integral = -self.windup_guard

        # derivative
        if self.last_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0

        out = self.kp * error + self.ki * self.integral + self.kd * derivative

        # store for next time
        self.last_error = error
        self.last_time = now

        return out
