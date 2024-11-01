#imports
#from ulab import numpy as np use for real
import numpy as np #testing only
import grid_maker as gm
import heapq 
# ***********************for programming grid*******************************************
start_point = 8 #put which number it is
#for start points
#   0,1  ,2 ,3
#13[] [] [] []4
#14[] [] [] []5
#15[] [] [] []6
#16[] [] [] []7
#17[] [] [] []8
#  9,10,11,12

blocks = ((2,2,2),(2,2,2))#rows,colums,spot based on below|cannot have just one value
#  1
# [][][]
#4[][][]2
# [][][]
#  3
gates = ((0,0),(4,3))#row,colums based on above track|cannot just have one value
last_gate = (1,1)#row,colums based on above track
end_point = (0,0)#row,colums based on
#0, 1  ,2 ,3
#[] [] [] []0
#[] [] [] []1
#[] [] [] []2
#[] [] [] []3
#[] [] [] []4
#****************************************************************************************

grid = gm.make_changes(start_point,end_point,gates,last_gate,blocks)#making and assinging the grid
gm.print_matrix(grid)
#variables
commands = ()



