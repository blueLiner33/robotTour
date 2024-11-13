#make if nesscarcy for the multthreading and holding them to put into main.
import movement as move
import _thread
import time
import PID as pid
import main


lock = _thread.allocate_lock()

data_list = []#keeps tracking of incoming data|needs to be shared across the modules

def core_one_thread():#runs on second core to always update other systems 
    global lock
    global data_list
    while True:
        lock.acquire()
        data_list = move.parsedata()
        move.RightMotor_pid.update(data_list)#needs to be fixed
        move.LeftMotor_pid.update(data_list)#needs to be fixed
        move.RightMotor_pid.adjust()
        move.LeftMotor_pid.adjust()
        time.sleep(0.5)#ajust delay as needed need for stablity I think
        lock.release()
complete = False
current_movement = None
def core_two():#main loop used for movement
    global lock
    global data_list
    global complete
    global current_movement
    while True:
        lock.acquire()
        move.start_forward(data_list)
        move.stop()
        current_movement = 0
        if current_movement == 1:
                move.forward(data_list)
        elif current_movement == 2:
                move.right(data_list)
        elif current_movement == 3:
                move.left(data_list)
        elif current_movement == 4:
                move.oneeighty(data_list)
        else:
                pass