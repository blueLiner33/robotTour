robot
│── main.py         # High-level program (commands)
│
├── config.py       # Constants only: pins, wheel size, pulses, speeds 
│
├── sensor_fusion.py     #position tracking and sensor combining
│
├── encoders.py     #encoder class and reading
│
├── imu.py          #imu class and reading
│
├── servo.py        #water bottle control
│
├── PID.py          #adjusts pwm based on goals
│
├── motors.py       #low level motor control
