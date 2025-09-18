import time
from machine import Pin

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