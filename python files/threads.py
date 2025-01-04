import movement as move
import _thread
import time
import read_sensors as resen

# Shared data and flags
data_list = []
lock = _thread.allocate_lock()
complete = False
current_movement = None

# Thread to handle sensor updates and motor adjustments on core one
def core_one_thread():
    global lock, data_list, complete
    while not complete:
        with lock:
            data_list = resen.get_data()
            # Format of data_list: rpm1, degrees_one, rpm2, degrees_two, yaw, x_accel, y_accel, z_accel
            if data_list:
                # Update and adjust motor PID values
                move.RightMotor_pid.update(data_list[0])  # RPM1
                move.LeftMotor_pid.update(data_list[2])   # RPM2
                move.RightMotor_pid.adjust()
                move.LeftMotor_pid.adjust()
        time.sleep(0.5)  # Prevent overloading the thread

# Main loop to process movement commands
#for testing
start_forward_executed = True
def process_command(movement):
    global current_movement, complete, start_forward_executed, data_list
    with lock:
        if data_list != None:
            if start_forward_executed == False:
                # Initialize forward movement using x_accel (data_list[5])
                move.start_forward(data_list[5])
                start_forward_executed = True
            else:
                if movement == 0:  # Stop
                    move.stop()
                elif movement == 1:  # Move forward
                    move.distance_moved(data_list[5])  # Update distance
                    move.forward(data_list)
                elif movement == 2:  # Turn right
                    move.right(data_list[4])  # Use yaw (data_list[4])
                elif movement == 3:  # Turn left
                    move.left(data_list[4])  # Use yaw (data_list[4])
                elif movement == 4:  # Perform 180-degree turn
                    move.oneeighty(data_list[4])  # Use yaw (data_list[4])
                time.sleep(0.1)  # Prevent overloading
            move.stop()  # Ensure the motors are stopped after processing
        else:
            pass