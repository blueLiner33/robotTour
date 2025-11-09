#encoders
PULSES_PER_REV = 240 
encoder_mode = "2x"

    #pins
ENCODER_A1 = 19  # Motor 1 encoder A pin
ENCODER_B1 = 18  # Motor 1 encoder B pin
ENCODER_A2 = 16   # Motor 2 encoder A pin
ENCODER_B2 = 17   # Motor 2 encoder B pin

#start switch
button_pin = 3 
switch = True # switch state

#imu
uart_tx_pin = 0
uart_rx_pin = 1

#motors
    #motor PID
kP = 0.25
Ki = 0.02
Kd = 0.001


    #motor pins 
RightMotor_m1 = 14
RightMotor_m2 = 15
RightMotor_pwm = 2

LeftMotor_m1 = 13
LeftMotor_m2 = 12
LeftMotor_pwm = 4

#servo
servo_pin = 21


#event_standards
grid_distance = 50 #cm
edge_distance = 30 #cm *for water bottle pickup
edge_distance = 25 #cm


#robot size
wheel_diameter = 70 #mm
span_wheels = 12#mm distance between wheels

#motors


#movement
encoder_merge_factor = .2 #amount of weight encoders have


#Kalman filter
Qx = 0.05      # Encoder noise for x
Rx = 0.1       # IMU noise for x
Qy = 0.05      # Encoder noise for y
Ry = 0.1       # IMU noise for y
Qtheta = 0.01  # Encoder noise for theta
Rtheta = 0.05  # IMU noise for theta

#PID
turn_kp = 2.0
turn_ki = 0.0
turn_kd = 0.05

forward_kp = 0.02
forward_ki = 0.0
forward_kd = 0.0