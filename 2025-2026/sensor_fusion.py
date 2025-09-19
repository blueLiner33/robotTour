#from movement.motors import car
import math
from config import encoder_merge_factor, wheel_diameter, PULSES_PER_REV, span_wheels, Qx, Rx, Qy, Ry, Qtheta,Rtheta
from encoders import encoder_right, encoder_left
from imu import rvc

class robot:
    def __init__(self,encoder_right,encoder_left,IMU):
        #shared
        self.xpos = 0
        self.ypos = 0
        self.theta = 0
        
        #imu values
        self.theta_imu = 0
        self.velocity_y = 0
        self.velocity_x = 0

        self.encoder_right = encoder_right
        self.encoder_left = encoder_left
        
        self.right_ticks = 0
        self.left_ticks = 0
        
        encoder_right.reset()
        encoder_left.reset()
        
        self.IMU = IMU
        
        #kalman
        self.x_est = 0
        self.y_est = 0
        self.theta_est = 0
        self.Px = 0
        self.Py = 0 
        self.Ptheta = 0 
        
    def calculate_position_IMU(self): #in m/2^s
        #y=y0+vyt+ayt^2
        #x=x0+vxt+axt^2
        try:
            imu_data = self.IMU.heading()
        except Exception as e:
            return self.xpos, self.ypos, self.theta  
        
        deltaTime = imu_data[1] / 1000
        accel_X = imu_data[0][3]
        accel_Y = imu_data[0][4]
        theta_imu = imu_data[0][0]#might not be cont.

        
        current_x = (self.xpos+self.velocity_x*deltaTime+(1/2)*accel_X*(deltaTime**2))*1000
        self.velocity_x = self.velocity_x+accel_X*deltaTime   #might need to change to other eqation
        
        current_y = (self.ypos+self.velocity_y*deltaTime+(1/2)*accel_Y*(deltaTime**2))*1000
        self.velocity_y = self.velocity_y+accel_Y*deltaTime #might need to change to other eqation

        return current_x,current_y,theta_imu

    def calculate_position_encoder (self):
        try:
            ticks_right = self.encoder_right.read()
            ticks_left = self.encoder_left.read()
        except Exception as e:
            return self.xpos, self.ypos, self.theta

        d_right = (2 * math.pi * (wheel_diameter / 2)) / PULSES_PER_REV * (ticks_right - self.right_ticks)
        d_left = (2 * math.pi * (wheel_diameter / 2)) / PULSES_PER_REV * (ticks_left - self.left_ticks)

        
        self.right_ticks = ticks_right
        self.left_ticks = ticks_left
        
        delta_theta = (d_right - d_left) / span_wheels
        d_center = (d_right+d_left)/2
        
        xpos = self.xpos +d_center*math.cos(self.theta+(delta_theta/2))
        ypos = self.ypos +d_center*math.sin(self.theta+(delta_theta/2))
        
        theta = delta_theta+self.theta
        theta = (theta + math.pi) % (2 * math.pi) - math.pi

        return xpos, ypos, theta


    def calculate_position(self):

        IMU = self.calculate_position_IMU()       # [x, y, theta] 
        Encoder = self.calculate_position_encoder()  # [x, y, theta]

        #x
        self.Px += Qx
        Kx = self.Px / (self.Px + Rx)
        self.x_est = Encoder[0] + Kx * (IMU[0] - Encoder[0])
        self.Px = (1 - Kx) * self.Px

        #y
        self.Py += Qy
        Ky = self.Py / (self.Py + Ry)
        self.y_est = Encoder[1] + Ky * (IMU[1] - Encoder[1])
        self.Py = (1 - Ky) * self.Py

        #theta
        self.Ptheta += Qtheta
        Ktheta = self.Ptheta / (self.Ptheta + Rtheta)
        self.theta_est = Encoder[2] + Ktheta * (IMU[2] - Encoder[2])
        self.Ptheta = (1 - Ktheta) * self.Ptheta

        return self.x_est, self.y_est, self.theta_est