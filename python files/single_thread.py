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
            print(f"Processing command {movement}, index {command_index}")
            print(data_list[4]-turns)
            if not start_forward_executed:
                move.start_forward(sensor_data[5])
                start_forward_executed = True
            elif movement == 0:  # Stop
                move.stop()
                command_index += 1
            elif movement == 1:  # Forward
                move.distance_moved(sensor_data[5])
                move.forward(sensor_data[5])
            elif movement == 2:  # Right
                if move.right(sensor_data[4] - turns):
                    command_index += 1
                    turns += 90
            elif movement == 3:  # Left
                if move.left(sensor_data[4] - turns):
                    command_index += 1
                    turns -= 90
            elif movement == 4:  # 180-degree turn
                if move.oneeighty(sensor_data[4] - turns):
                    command_index += 1
                    turns -= 180
            else:
                command_index += 1
        else:
           
            complete = True
            print("commands done")
            move.stop()

        await asyncio.sleep(0.01)

    print("loop stopped")


def start_main_loop(commands):
    """
    Start the sensor data task and the asyncio main loop.
    """
    global complete
    complete = False

    try:
        loop = asyncio.get_event_loop()
        loop.create_task(sensor_data_task())  # Start sensor data acquisition task
        loop.run_until_complete(main_loop(commands))  # Run the main loop
    except Exception as e:
        print(f"Error: {e}")
    finally:
        complete = True

