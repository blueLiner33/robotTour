import movement as move
import _thread
import time
import read_sensors as resen
# Shared data and flags
data_list = []
lock = _thread.allocate_lock()
complete = False
current_movement = None
def core_one_thread():#runs on second core to always update other systems 
    global lock, data_list
    while not complete:
        with lock:
            data_list = resen.get_data()
            if data_list:
                move.RightMotor_pid.update(data_list)#needs to be fixed
                move.LeftMotor_pid.update(data_list)#needs to be fixed
                move.RightMotor_pid.adjust()
                move.LeftMotor_pid.adjust()
                time.sleep(0.5)#adjust

def process_command(movement):#main loop used for movement
    global current_movement, complete
    with lock:
        if movement == 0:
            move.stop()
        elif movement == 1:
            move.forward(data_list)
        elif movement == 2:
            move.right(data_list)
        elif movement == 3:
            move.left(data_list)
        elif movement == 4:
            move.oneeighty(data_list)
        time.sleep(0.1)#adjust
    move.stop()