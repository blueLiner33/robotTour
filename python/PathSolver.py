# ***********************for programming grid*******************************************
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


#imports
import numpy as np
import GridMaker as gm
print(gm.makeChanges(startPoint,endPoint,gates,lastGate,blocks))

#variables
commands = ()

def solvePath(grid):
    pass

