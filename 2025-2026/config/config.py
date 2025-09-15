from machine import UART, Pin, Timer

#encoders
PULSES_PER_REV = 240 
DEGREES_PER_COUNT = 360.0 / PULSES_PER_REV  

    #pins
ENCODER_A1 = 12  # Motor 1 encoder A pin
ENCODER_B1 = 11  # Motor 1 encoder B pin
ENCODER_A2 = 17   # Motor 2 encoder A pin
ENCODER_B2 = 18   # Motor 2 encoder B pin

#start switch
button = Pin(3, Pin.IN, Pin.PULL_UP)
switch = False # switch state

#imu
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) 

#motor pins 
RightMotor_m1,RightMotor_m2,RightMotor_pwm = 14,13,15
LeftMotor_m1,LeftMotor_m2,LeftMotor_pwm = 19,20,16

#servo
servo_pin = 21