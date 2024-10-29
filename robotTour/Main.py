# ***********************for programming grid*******************************************
GoalTime = 0
startPoint = 17 #put which number it is
#for start points
#   0,1  ,2 ,3
#13[] [] [] []4
#14[] [] [] []5
#15[] [] [] []6
#16[] [] [] []7
#17[] [] [] []8
#  9,10,11,12

blocks = ((0,0,1),(2,2,4))#rows,colums,spot based on below
#  1
# [][][]
#4[][][]2
# [][][]
#  3
gates = ((0,0),(1,1))#row,colums based on above track
lastGate = (1,1)#row,colums based on above track
endPoint = (0,0)#row,colums based on 
# 0,1  ,2 ,3
#[] [] [] []0
#[] [] [] []1
#[] [] [] []2
#[] [] [] []3
#[] [] [] []4
#****************************************************************************************

#**************************dependencys***************************************************

#setting imputs
import PathSolver as Path
import GridMaker as Grid
import Movement as Move

Course = Grid.makeChanges(startPoint,endPoint,gates,lastGate,blocks)
commands = Path.solvePath(Course)
speed = Move.getmovementSpeed(GoalTime,commands)
#****************************************************************************************


#***********************************Main Code********************************************
#starting movement
Move.forward(.5,Move.speed)

#make sure motors are synced 
#_thread.start_new_thread(Move.BothMotorPos, ())#starting new thread


for command in Move.MotorCommands():
    if command == 1:
        Move.left(1,speed)
    elif command == 2:
        Move.right(1,speed)
    elif command == 3:
        Move.forward(1,speed)
    else:
        pass
Move.MotorPower.off()#redudant but to make sure
