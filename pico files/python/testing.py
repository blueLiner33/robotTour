#imports
#import grid_maker as gm
import path_solver as ps
#import heaper as heap

# ***********************for programming grid*******************************************
#add when it has to be manual solve
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
#****************************************************************************************

#grid = gm.make_changes(start_point,end_point,gate_points,last_gate_point,blocks)#making grid
#gm.print_matrix(grid)
print(ps.give_commands(start_point,end_point,gate_points,last_gate_point,blocks))
