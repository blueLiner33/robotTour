robot/
│── main.py               # High-level program (game-day logic)
│
├── config/
│   └── config.py         # Constants only: pins, wheel size, pulses, speeds
│
├── movement/
│   ├── __init__.py       # empty or convenience imports
│   ├── motors.py         # low-level motor wrapper
│   ├── encoders.py       # low-level encoder wrapper
│   ├── movement.py       # high-level movement primitives (cells, turns)
│   └── servo.py          # servo wrapper for bottles
│
├── sensors/
│   ├── __init__.py       # empty or convenience imports
│   └── imu.py            # IMU wrapper
│
└── utils/
    ├── __init__.py       # empty or convenience imports
    └── helpers.py        # math, unit conversions, filtering
