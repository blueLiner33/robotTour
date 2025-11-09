# movement.py
import time
import math
from config import grid_distance, wheel_diameter, PULSES_PER_REV, span_wheels, encoder_merge_factor, kP, Ki, Kd
from encoders import encoder_right, encoder_left
from imu import rvc
from sensorFusion import robot as SFRobot
from motors import RightMotor, LeftMotor
from PID import PID

# helper conversions
def mm_per_tick():
    circ = math.pi * wheel_diameter  # mm
    return circ / PULSES_PER_REV

# sensor fusion instance
sf = SFRobot(encoder_right, encoder_left, rvc)

# heading PID for straight driving
heading_pid = PID(kp=0.8, ki=0.0, kd=0.02, windup_guard=0.5)

def reset_encoders():
    encoder_right.reset()
    encoder_left.reset()
    sf.right_ticks = 0
    sf.left_ticks = 0

def trapezoidal_speed(progress, target, base_speed, min_speed=0.3, max_speed=None):
    """
    Compute trapezoidal profile speed given progress (ticks) and target (ticks).
    """
    if max_speed is None:
        max_speed = base_speed

    accel_dist = 0.2 * target
    decel_dist = 0.2 * target

    if progress < accel_dist:  # accelerate
        speed = min_speed + (max_speed - min_speed) * (progress / accel_dist)
    elif progress > target - decel_dist:  # decelerate
        remain = max(1, target - progress)
        speed = min_speed + (max_speed - min_speed) * (remain / decel_dist)
    else:  # cruise
        speed = max_speed
    return max(min_speed, min(max_speed, speed))

def move_cells(n_cells, base_speed=0.6, timeout=5.0):
    """
    Move forward n_cells (positive forward) with trapezoidal speed profile.
    """
    if n_cells == 0:
        return

    target_mm = abs(n_cells) * grid_distance * 10.0
    per_tick_mm = mm_per_tick()
    target_ticks = target_mm / per_tick_mm
    direction_sign = 1 if n_cells > 0 else -1

    reset_encoders()
    heading_pid.reset()

    start_time = time.time()
    try:
        _, _, theta0 = sf.calculate_position()
    except Exception:
        theta0 = 0.0

    while True:
        if time.time() - start_time > timeout * max(1, abs(n_cells)):
            break

        tr = encoder_right.read()
        tl = encoder_left.read()
        avg_ticks = (abs(tr) + abs(tl)) / 2.0

        # trapezoidal speed target
        profile_speed = trapezoidal_speed(avg_ticks, target_ticks, base_speed)

        # heading correction
        try:
            _, _, theta = sf.calculate_position()
        except Exception:
            theta = theta0
        err = (theta0 - theta + math.pi) % (2 * math.pi) - math.pi

        corr = heading_pid.update(err)
        corr_limit = profile_speed * 0.6
        corr = max(-corr_limit, min(corr, corr_limit))

        left_speed = direction_sign * profile_speed + corr
        right_speed = direction_sign * profile_speed - corr

        # clamp
        def clamp(v):
            return max(-1.0, min(1.0, v))

        LeftMotor.set_speed(clamp(left_speed))
        RightMotor.set_speed(clamp(right_speed))

        if avg_ticks >= target_ticks:
            break

        time.sleep(0.005)

    LeftMotor.stop()
    RightMotor.stop()
    time.sleep(0.02)

def rotate_angle_deg(angle_deg, base_speed=0.5, timeout=2.0):
    """
    Rotate robot by angle_deg (positive = CCW) with smooth trapezoid speed.
    """
    heading_pid.reset()
    angle_rad = math.radians(angle_deg)

    try:
        _, _, theta0 = sf.calculate_position()
    except Exception:
        theta0 = 0.0

    target_theta = theta0 + angle_rad
    target_theta = (target_theta + math.pi) % (2 * math.pi) - math.pi

    start = time.time()
    while True:
        if time.time() - start > timeout:
            break

        try:
            _, _, theta = sf.calculate_position()
        except Exception:
            theta = theta0

        err = (target_theta - theta + math.pi) % (2 * math.pi) - math.pi
        if abs(err) < math.radians(1.5):
            break

        # trapezoid profile based on remaining angle
        remain = abs(err)
        max_speed = base_speed
        min_speed = 0.25
        accel_angle = math.radians(20)
        decel_angle = math.radians(20)

        if remain > decel_angle:
            speed = max_speed
        else:
            speed = min_speed + (max_speed - min_speed) * (remain / decel_angle)
            if speed < min_speed:
                speed = min_speed

        rot = max(-1.0, min(1.0, speed if err > 0 else -speed))

        LeftMotor.set_speed(rot)
        RightMotor.set_speed(-rot)

        time.sleep(0.01)

    LeftMotor.stop()
    RightMotor.stop()
    time.sleep(0.05)

def turn_left_90():
    rotate_angle_deg(90.0, base_speed=0.6, timeout=1.5)

def turn_right_90():
    rotate_angle_deg(-90.0, base_speed=0.6, timeout=1.5)

# servo actions
from servo import grap_latch
def pickup():
    grap_latch.open()
    time.sleep(0.2)
    grap_latch.close()
    time.sleep(0.2)

def drop():
    grap_latch.open()
    time.sleep(0.2)
    grap_latch.close()
    time.sleep(0.2)
