import uasyncio as asyncio
import read_sensors as resen
import movement as move


data_list = []
complete = False
current_movement = None


async def sensor_data_task():
    global data_list
    while not complete:
        data_list = resen.get_data()
        await asyncio.sleep(0.01) 


async def main_loop(commands):
    global complete, current_movement

    command_index = 0
    start_forward_executed = False
    turns = 0

    while not complete:
        sensor_data = data_list

        if not sensor_data:
            print("no sensor data")
            await asyncio.sleep(0.01)  
            continue

        #PID
        move.RightMotor_pid.update(sensor_data[0])  # RPM1
        move.LeftMotor_pid.update(sensor_data[2])   # RPM2
        move.RightMotor_pid.adjust()
        move.LeftMotor_pid.adjust()

        #command process
        if command_index < len(commands):
            movement = commands[command_index]
            current_movement = movement
            #print(f"Processing command {movement}, index {command_index}") #for testing what command it is doing
            if not start_forward_executed:
                if move.start_forward(move.get_distance_traveled()) != True:
                    move.distance_moved(sensor_data[6],(resen.get_update_time()))
                    move.start_forward(move.get_distance_traveled())
                else:
                    start_forward_executed = True
            elif movement == 0:  # Stop
                move.stop()
                command_index += 1
            elif movement == 1:  # Forward
                move.distance_moved(sensor_data[6],(resen.get_update_time()))
                move.forward(move.get_distance_traveled())
            elif movement == 2:  # Right
                if move.right(sensor_data[4] - (turns*90)):
                    command_index += 1
                    turns += 1
            elif movement == 3:  # Left
                if move.left(sensor_data[4] - (turns*90)):
                    command_index += 1
                    turns -= 1
            elif movement == 4:  # 180
                if move.oneeighty(sensor_data[4] - (turns*90)):
                    command_index += 1
                    turns -= 2
            else:
                command_index += 1
        else:
            
            complete = True
            print("commands done")
            move.stop()

        await asyncio.sleep(0.01)

    print("loop stopped")


def start_main_loop(commands):
    
    global complete
    complete = False

    try:
        loop = asyncio.get_event_loop()
        loop.create_task(sensor_data_task())  
        loop.run_until_complete(main_loop(commands))  
    except Exception as e:
        print(f"Error: {e}")
    finally:
        complete = True