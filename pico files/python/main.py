# ***********************for programming grid*******************************************
target_time = 0 #the target time
start_point = 2 #put which number it is
#for start points
#   0,1  ,2 ,3
#13[] [] [] []4
#14[] [] [] []5
#15[] [] [] []6
#16[] [] [] []7
#17[] [] [] []8
#  9,10,11,12

blocks = ((2,2,2),(2,2,3))#rows,colums,spot based on below|cannot have just one value
#  1
# [][][]
#4[][][]2
# [][][]
#  3
gate_points = ((0,0),(4,3),(2,2))#row,colums based on above track

#cannot just have one value
#excluded any gates that are reached through starting movement of forward(.5)

last_gate_point = (1,1)#row,colums based on above track
end_point = (3,3)#row,colums based on
#0, 1  ,2 ,3
#[] [] [] []0
#[] [] [] []1
#[] [] [] []2
#[] [] [] []3
#[] [] [] []4
#**************************dependency***************************************************
#setting imputs
import path_solver as path
import movement as move
import threads
import _thread
import time
commands = path.give_commands(start_point,end_point,gate_points,last_gate_point,blocks)
if len(commands) >= 30:
    raise ValueError ('manual solve most likely needed commands greater than 30')
else:
    pass

#***********************************Main Code********************************************
#might need to add a delay.
_thread.start_new_thread(threads.core_one_thread, ())
while threads.current_movement == None:
    pass
else:
    for movement in commands:
        if movement == 0 and threads.complete == False:
            threads.current_movement = 1
            move.stop()
            
        elif movement == 1:
            threads.current_movement = 2
            move.stop()
        elif movement == 2:
            threads.current_movement = 3
            move.stop()
        elif movement == 3:
            threads.current_movement = 4
            move.stop()
        else:
            pass
move.stop()
time.sleep(100000)

#for threading
#move.lock(to avoid corruption)

