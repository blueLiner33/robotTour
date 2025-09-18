#encoders
PULSES_PER_REV = 240 
encoder_mode = "2x"

    #pins
ENCODER_A1 = 12  # Motor 1 encoder A pin
ENCODER_B1 = 11  # Motor 1 encoder B pin
ENCODER_A2 = 17   # Motor 2 encoder A pin
ENCODER_B2 = 18   # Motor 2 encoder B pin

#start switch
button_pin = 3 
switch = False # switch state

#imu
uart_tx_pin = 0
uart_rx_pin = 1

#motors
    #motor PID
kP = 0          # proportional constant
Ki = 0          # integral constant
Kd = 0          # derivative constant
    #motor pins 
RightMotor_m1 = 14
RightMotor_m2 = 13
RightMotor_pwm = 15

LeftMotor_m1 = 19
LeftMotor_m2 = 20
LeftMotor_pwm = 16

#servo
servo_pin = 21


#event_standards
grid_distance = 50 #cm
edge_distance = 30 #cm *for water bottle pickup
edge_distance = 25 #cm


#robot size
wheel_diameter = 50 #mm
span_wheels = 10 #mm distance between wheels


#motors


#movement
encoder_merge_factor = .2 #amount of weight encoders have
