import numpy as np
grid = np.ones((5,4,3,3))

def printGrid():#helpful for debugging
    for i in range(grid.shape[0]):  
        matrix_lines = [""] * 3  
        for j in range(grid.shape[1]):  
            for k in range(3):  
                line = " | ".join(f"{int(value):2}" for value in grid[i, j, k])  
                matrix_lines[k] += line + "   "  

        for line in matrix_lines:
            print(line)
        print("\n" + "-" * 80)  


def gateChange(gates,lastGate):#changes grid based on both gates and last gate
    #normal gates
    for i in range (len(gates)):
        grid[gates[i-1][0],gates[i-1][1],1,1] = 4
    grid[lastGate[0],lastGate,1,1] = 5


def startChange(startPoint):#changes grid based on start point
    #makes all the points on the outside 0
    
    #does top and bottom
    for col in range (4):
        for size in range(3):
            grid[0,col,0,size] = 0
            grid[4,col,2,size] = 0
    #does sides
    for row in range(5):
        for size in range(3):
            grid[row,0,size,0] = 0
            grid[row,3,size,2] = 0
            
    #adds the start point
    if (3>= startPoint) and (startPoint >= 0):
        grid[0,startPoint,0,1] = 2
    if (startPoint >= 4) and (startPoint<=8):
        grid[startPoint-4,3,1,2] = 2
    if (startPoint >=9) and (startPoint<=12):
        grid[4,startPoint-9,2,1] = 2
    if (startPoint>=13) and (startPoint<=17):
        grid[startPoint-13,0,1,0] = 2
    else:
        print('error in value of startPoint')


def blockChange(blocks):#changes grid based on blocks
    for i in range(len(blocks)):
        if blocks[i-1][2] == 1:#top
            grid[blocks[i-1][0],[blocks[i-1][1]],0,(0,1,2)] = 0
        if blocks[i-1][2] == 2:#right side
            grid[blocks[i-1][0],[blocks[i-1][1]],(0,1,2),2] = 0
        if blocks[i-1][2] == 3:#bottom
            grid[blocks[i-1][0],[blocks[i-1][1]],2,(0,1,2)] = 0
        if blocks[i-1][2] == 4:#left side
            grid[blocks[i-1][0],[blocks[i-1][1]],(0,1,2),0] = 0


def makeChanges(startPoint,endPoint,gates,lastGate,blocks):#makes changes to the grid based on the points
    #makes start change
    startChange(startPoint)
    #makes end change
    grid[endPoint[0],endPoint[1],1,1] = 3
    #makes block change
    gateChange(gates,lastGate)
    blockChange(blocks)
    return grid.reshape(12,15)