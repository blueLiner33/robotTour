import time

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0, sample_time=0.01, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._last_time = time.ticks_ms()
        self._last_error = 0
        self._integral = 0

        self.output_limits = output_limits  # (min, max)

    def compute(self, feedback):
        """Calculate PID output given current feedback value"""
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time) / 1000.0
        if dt < self.sample_time:
            return None  # don’t compute yet

        error = self.setpoint - feedback
        self._integral += error * dt
        derivative = (error - self._last_error) / dt if dt > 0 else 0

        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative

        # Apply limits
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)

        # Save for next iteration
        self._last_time = now
        self._last_error = error

        return output
