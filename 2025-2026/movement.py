#from movement.motors import car
import math
from config import encoder_merge_factor, wheel_diameter, PULSES_PER_REV, span_wheels
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
    def calculate_position_IMU(self): #in m/2^s
        #y=y0+vyt+ayt^2
        #x=x0+vxt+axt^2
        deltaTime = ((self.IMU.heading())[1])/1000
        accel_X = (self.IMU.heading())[0][3]
        accel_Y = (self.IMU.heading())[0][4]
        theta_imu = (self.IMU.heading())[0][0] #check if cont. 
        
        current_x = (self.xpos+self.velocity_x*deltaTime+(1/2)*accel_X*(deltaTime**2))/1000
        self.velocity_x = self.velocity_x+accel_X*deltaTime   #might need to change to other eqation
        
        current_y = (self.ypos+self.velocity_y*deltaTime+(1/2)*accel_Y*(deltaTime**2))/1000
        self.velocity_y = self.velocity_y+accel_Y*deltaTime #might need to change to other eqation

        return current_x,current_y,theta_imu

    def calculate_position_encoder (self):
        ticks_right = self.encoder_right.read()  
        ticks_left  = self.encoder_left.read()
        
        d_right = (2*3.1415926535*(wheel_diameter/2))/PULSES_PER_REV*(self.encoder_right-self.right_ticks)
        d_left = (2*3.1415926535*(wheel_diameter/2))/PULSES_PER_REV*(self.encoder_left-self.left_ticks)
        
        self.right_ticks = ticks_right
        self.left_ticks = ticks_left
        
        delta_theta = (d_right - d_left) / span_wheels
        d_center = (d_right+d_left)/2
        
        xpos = self.xpos +d_center*math.cos(self.theta+(delta_theta/2))
        ypos = self.ypos +d_center*math.sin(self.theta+(delta_theta/2))
        
        theta = delta_theta+self.theta
        
        return xpos, ypos, theta

    def calculate_position(self):
        IMU = self.calculate_position_IMU()
        Encoder = self.calculate_position_encoder()

        self.xpos = IMU[0]*(1-encoder_merge_factor)+Encoder[0]*encoder_merge_factor
        self.ypos = IMU[1]*(1-encoder_merge_factor)+Encoder[1]*encoder_merge_factor
        self.theta = IMU[2]*(1-encoder_merge_factor)+Encoder[2]*encoder_merge_factor

        return self.xpos,self.ypos,self.theta