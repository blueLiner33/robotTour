#import time
#from machine import Pin

#from servo import grab_latch
'''
button = Pin(button_pin, Pin.IN, Pin.PULL_UP)

while True:
    if button.value() == 0:
        switch = True
        break
    else:
        pass
    time.sleep(0.1)
'''


#format of commands
#auto go forward half starting distnace then commands of go forward, turn right, turn left. All have no variable. 
# But the command pickupwater bottle does value one of the following 
#  1
# [][][]
#4[][][]2
# [][][]
#  3
#then drop water bottle will be full movement into square. 
#then next movement out might have to calculated differently. 







#grab_latch.close()

##example
# main.py
import time
from machine import Pin
from config import button_pin
from movement import move_cells, turn_left_90, turn_right_90, pickup, drop
from movement import sf  # sensor fusion instance

# start button (active low)
button = Pin(button_pin, Pin.IN, Pin.PULL_UP)

# Example command list: this is where you put the contest commands
# Use: "F" forward one cell, "L" turn left, "R" turn right, "P" pickup, "D" drop
# Example scenario: forward 1, right, forward 2, pickup, turn around, forward 2, drop
commands = ["F1", "R", "F2", "P", "R", "R", "F2", "D"]

def wait_for_start():
    print("Waiting for start button press...")
    while button.value() == 1:
        time.sleep(0.05)
    # debounce
    time.sleep(0.2)
    while button.value() == 0:
        time.sleep(0.01)
    print("Starting run...")

def parse_and_run(cmds):
    for c in cmds:
        if c.startswith("F"):
            try:
                n = int(c[1:])
            except Exception:
                n = 1
            print("Forward", n)
            # speed and timeout may be tuned
            move_cells(n_cells=n, base_speed=0.7, timeout=5.0)
        elif c == "L":
            print("Turn left 90")
            turn_left_90()
        elif c == "R":
            print("Turn right 90")
            turn_right_90()
        elif c == "P":
            print("Pickup")
            pickup()
        elif c == "D":
            print("Drop")
            drop()
        else:
            print("Unknown command:", c)
        # small pause between commands
        time.sleep(0.05)

if __name__ == "__main__":
    wait_for_start()
    # optional: zero pose/encoders before run
    sf.encoder_right.reset()
    sf.encoder_left.reset()
    sf.right_ticks = 0
    sf.left_ticks = 0
    parse_and_run(commands)
    print("Run completed")
