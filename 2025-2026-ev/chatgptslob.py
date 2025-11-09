"""
navigation_patch.py

Contains:
- corrected calculate_position_IMU method (replace the existing method in sensor_fusion.robot class with this one)
- path planner utilities (compute C, entry/exit points)
- path generator that returns waypoints (A -> entry -> arc samples -> exit -> B)
- simple differential-drive waypoint follower using PID for heading and a proportional forward controller

Units: all positions and distances in centimeters (cm).

Usage:
1) Put this file on the Pico (same directory or in your project).
2) Import functions or copy the calculate_position_IMU body into your sensor_fusion.robot class replacing the old function.
3) Call plan_and_run(A,B,c_offset_cm,clearance_cm,robot_instance)

Note: This is a practical, simple controller. Tune the PID and speeds for your robot.
"""

import math
import time
from PID import PID
from motors import RightMotor, LeftMotor

# --- Utility math ---

def _wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi

# --- Corrected IMU-based position function
# Replace the calculate_position_IMU method in your sensor_fusion.robot class with the following function.
def calculate_position_IMU_fixed(self):
    """
    This function assumes:
    - self.xpos, self.ypos are stored in centimeters (cm).
    - self.velocity_x and self.velocity_y are stored in meters/second (m/s).
    - imu.heading() returns ((yaw, pitch, roll, ax, ay, az), dt_seconds)
    - accelerations ax, ay are in m/s^2.

    It returns (x_cm, y_cm, theta_deg)
    """
    try:
        imu_data = self.IMU.heading()
    except Exception:
        # If IMU read fails return previous estimate
        return self.xpos, self.ypos, self.theta

    # imu_data: ( (yaw, pitch, roll, ax, ay, az), dt_seconds )
    dt = imu_data[1]
    if dt is None or dt <= 0:
        dt = 0.0

    yaw = imu_data[0][0]  # degrees
    accel_X = imu_data[0][3]  # m/s^2 (body frame)
    accel_Y = imu_data[0][4]  # m/s^2

    # Integrate using current velocities (which are stored in m/s)
    # delta_x in meters
    delta_x_m = self.velocity_x * dt + 0.5 * accel_X * (dt ** 2)
    delta_y_m = self.velocity_y * dt + 0.5 * accel_Y * (dt ** 2)

    # update velocities
    self.velocity_x = self.velocity_x + accel_X * dt
    self.velocity_y = self.velocity_y + accel_Y * dt

    # convert delta meters to centimeters
    delta_x_cm = delta_x_m * 100.0
    delta_y_cm = delta_y_m * 100.0

    self.xpos = self.xpos + delta_x_cm
    self.ypos = self.ypos + delta_y_cm

    # normalize yaw to [-180,180]
    yaw = (yaw + 180.0) % 360.0 - 180.0

    return self.xpos, self.ypos, math.radians(yaw)

# --- Path planning ---

def compute_midpoint(A, B):
    return ((A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0)


def unit_vector(A, B):
    dx = B[0] - A[0]
    dy = B[1] - A[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return (1.0, 0.0)
    return (dx / length, dy / length)


def perpendicular(u):
    # rotate 90 degrees CCW
    return (-u[1], u[0])


def compute_C_from_midpoint(A, B, lateral_offset_cm):
    """Compute coordinates of C given A,B and lateral_offset in cm.
    lateral_offset_cm positive moves C to the left of AB (when looking from A to B).
    """
    M = compute_midpoint(A, B)
    u = unit_vector(A, B)
    n = perpendicular(u)
    Cx = M[0] + lateral_offset_cm * n[0]
    Cy = M[1] + lateral_offset_cm * n[1]
    return (Cx, Cy)


def compute_entry_exit_points(A, B, C, R):
    """Return entry and exit points along AB where the line intersects circle center C radius R.
    A,B,C are (x,y) in cm. R in cm.
    Returns (entry_point, exit_point) or None if no intersection.
    """
    u = unit_vector(A, B)
    # solve ||A + u*t - C||^2 = R^2 for t (distance along AB from A)
    Ax, Ay = A
    Cx, Cy = C
    ux, uy = u

    # components for quadratic at^2 + bt + c = 0
    a = 1.0  # u is unit
    b = 2 * (ux * (Ax - Cx) + uy * (Ay - Cy))
    c = (Ax - Cx) ** 2 + (Ay - Cy) ** 2 - R ** 2

    disc = b * b - 4 * a * c
    if disc < 0:
        return None

    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2 * a)
    t2 = (-b + sqrt_disc) / (2 * a)
    # Convert t to points
    P1 = (Ax + ux * t1, Ay + uy * t1)
    P2 = (Ax + ux * t2, Ay + uy * t2)
    # ensure t1 < t2
    if t1 <= t2:
        entry, exit = P1, P2
    else:
        entry, exit = P2, P1
    return entry, exit


def generate_arc_points(C, P_entry, P_exit, num_samples=16):
    """Generate points along the circular arc from P_entry to P_exit around center C.
    Chooses the shorter arc that keeps the clearance side consistent.
    """
    Cx, Cy = C
    ex, ey = P_entry[0] - Cx, P_entry[1] - Cy
    ox, oy = P_exit[0] - Cx, P_exit[1] - Cy
    r = math.hypot(ex, ey)
    a1 = math.atan2(ey, ex)
    a2 = math.atan2(oy, ox)

    # produce arc from a1 to a2 choosing the direction that goes the long way around the obstacle's side
    # compute delta angle in range [-pi, pi]
    da = _wrap_angle(a2 - a1)
    # If da==0 then points coincide; still return single point
    points = []
    for i in range(num_samples + 1):
        frac = i / float(num_samples)
        theta = a1 + da * frac
        x = Cx + r * math.cos(theta)
        y = Cy + r * math.sin(theta)
        points.append((x, y))
    return points


def plan_path(A, B, lateral_offset_cm, clearance_cm, arc_samples=16):
    """Plan a path from A to B that goes around C.
    A,B: tuples in cm. lateral_offset_cm: signed lateral offset for C from midpoint.
    clearance_cm: radius of circular clearance around C.
    Returns list of waypoints [(x,y), ...] in cm.
    """
    C = compute_C_from_midpoint(A, B, lateral_offset_cm)
    pts = compute_entry_exit_points(A, B, C, clearance_cm)
    if pts is None:
        # fallback: if AB does not intersect the clearance circle, we will create a wide pass
        # compute the projection of C onto AB
        u = unit_vector(A, B)
        Ax, Ay = A
        proj_t = ( (C[0] - Ax) * u[0] + (C[1] - Ay) * u[1] )
        proj = (Ax + u[0] * proj_t, Ay + u[1] * proj_t)
        # set entry/exit symmetric around projection by clearance_cm
        entry = (proj[0] - u[0] * clearance_cm, proj[1] - u[1] * clearance_cm)
        exit = (proj[0] + u[0] * clearance_cm, proj[1] + u[1] * clearance_cm)
    else:
        entry, exit = pts

    arc_pts = generate_arc_points(C, entry, exit, num_samples=arc_samples)
    waypoints = []
    waypoints.append(A)
    waypoints.append(entry)
    waypoints.extend(arc_pts)
    waypoints.append(exit)
    waypoints.append(B)
    return waypoints, C

# --- Simple follower ---

class WaypointFollower:
    def __init__(self, robot_instance, base_speed=0.4, turn_scale=0.8):
        """
        robot_instance: an instance of your sensor_fusion.robot class (provides calculate_position())
        base_speed: nominal forward speed [0..1]
        turn_scale: scale factor for turning control
        """
        self.robot = robot_instance
        # PIDs
        self.turn_pid = PID(kp=2.0, ki=0.0, kd=0.05, windup_guard=1.0)
        self.forward_kp = 0.02
        self.base_speed = base_speed
        self.turn_scale = turn_scale

    def goto_point(self, target, pos_tolerance_cm=2.0, angle_tolerance_rad=0.1, timeout_s=10.0):
        start_ms = time.ticks_ms()
        while True:
            # time check
            elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            if elapsed > timeout_s:
                break

            pose = self.robot.calculate_position()
            x, y, theta = pose[0], pose[1], pose[2]
            dx = target[0] - x
            dy = target[1] - y
            dist = math.hypot(dx, dy)
            if dist <= pos_tolerance_cm:
                break

            desired_heading = math.atan2(dy, dx)
            heading_error = _wrap_angle(desired_heading - theta)

            # Turn control
            turn_u = self.turn_pid.update(heading_error)
            # Map turn to motor differential
            # forward speed scaling by distance
            forward_speed = self.base_speed
            if dist < 30.0:
                # slow down when close
                forward_speed = max(0.12, self.base_speed * (dist / 30.0))

            left = forward_speed - self.turn_scale * turn_u
            right = forward_speed + self.turn_scale * turn_u

            # clamp
            left = max(-1.0, min(1.0, left))
            right = max(-1.0, min(1.0, right))

            RightMotor.set_speed(right)
            LeftMotor.set_speed(left)

            time.sleep_ms(30)

        # stop motors briefly
        RightMotor.stop()
        LeftMotor.stop()

    def follow_waypoints(self, waypoints, tol_cm=3.0):
        for wp in waypoints:
            self.goto_point(wp, pos_tolerance_cm=tol_cm)

# --- Convenience function ---

def plan_and_run_from_distance(A, AB_distance_cm, lateral_offset_cm, clearance_cm, robot_instance):
    """Compute B using A and AB_distance_cm along +x axis.
    A: (x,y) in cm.
    AB_distance_cm: scalar distance from A to B.
    lateral_offset_cm: signed offset for C from midpoint.
    clearance_cm: clearance radius around C.
    """
    B = (A[0] + AB_distance_cm, A[1])
    waypoints, C = plan_path(A, B, lateral_offset_cm, clearance_cm)
    follower = WaypointFollower(robot_instance)
    follower.follow_waypoints(waypoints)
    return waypoints, C(A, B, lateral_offset_cm, clearance_cm, robot_instance):
    """High-level function: plans path and runs it using the robot_instance.
    A,B: (x,y) in cm.
    lateral_offset_cm: signed lateral offset for C from midpoint (positive=left from A->B).
    clearance_cm: radius around C in cm.
    robot_instance: sensor_fusion.robot instance.
    """
    waypoints, C = plan_path(A, B, lateral_offset_cm, clearance_cm)
    follower = WaypointFollower(robot_instance)
    follower.follow_waypoints(waypoints)
    return waypoints, C

# Example usage block (commented)
if __name__ == '__main__':
    # quick dry-run example (replace robot_instance with a real instance)
    A = (0.0, 0.0)   # cm
    B = (200.0, 0.0) # cm  --> 2 meters
    lateral_offset_cm = 20.0  # C sits 20cm to left of midpoint
    clearance_cm = 30.0

    print('Planning path...')
    wps, C = plan_path(A, B, lateral_offset_cm, clearance_cm)
    print('Waypoints:', wps)
    print('C:', C)

    # To run on real robot:
    # from sensor_fusion import robot as RobotClass
    # rob = RobotClass(encoder_right, encoder_left, rvc)
    # plan_and_run(A, B, lateral_offset_cm, clearance_cm, rob)
