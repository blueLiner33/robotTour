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

commands = path.give_commands(start_point,end_point,gate_points,last_gate_point,blocks)
if len(commands) >= 30:
    raise ValueError ('manual solve most likely needed commands greater than 30')
else:
    pass
rpm = move.get_movement_speed(target_time,commands)
move.setting_motors(goal_rpm,rightMotor_current_rpm,leftMotor_current_rpm)#need to fix this here
#*
#*
#need a function that always returns the motors rpm and the posistion of the robot. Just needs to read the data that is already coming in 
#needs to use the threds feature.
#*
#*
#***********************************Main Code********************************************
def movement_loop():
    move.forward(0.5,rpm)
    move.stop()
    for movement in commands:
        if movement == 0:
            move.forward(0.5,rpm)
        elif movement == 1:
            move.rigth(rpm)
        elif movement == 2:
            move.left(rpm)
        elif movement == 3:
            move.oneeighty(rpm)
        else:
            pass
        move.stop()

movement_loop()#loop that moves the robot