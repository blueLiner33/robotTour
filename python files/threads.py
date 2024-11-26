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
            # format of data list rpm1,degrees_one,rpm2,degrees_two,yaw,x_accel,y_accel,z_accel
            if data_list:
                move.RightMotor_pid.update(data_list[0])
                move.LeftMotor_pid.update(data_list[2])
                move.RightMotor_pid.adjust()
                move.LeftMotor_pid.adjust()
                time.sleep(0.5)#adjust
start_forward_executed = False
def process_command(movement):#main loop used for movement
    global current_movement, complete, start_forward_executed
    with lock:
        if not start_forward_executed:
            move.start_forward(data_list[5])
            start_forward_executed = True  
        else:
            if movement == 0:
                move.stop()
            elif movement == 1:
                move.distance_moved(data_list[5])
                move.forward(data_list)
            elif movement == 2:
                move.right(data_list[4])
            elif movement == 3:
                move.left(data_list[4])
            elif movement == 4:
                move.oneeighty(data_list[4])
            time.sleep(0.1)#adjust
        move.stop()