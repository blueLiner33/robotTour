#from ulab import numpy as np use for real
import numpy as np#testing only
grid = np.zeros((15, 12))


def print_matrix(matrix):#testing function
    square_size = 3  
    rows, cols = matrix.shape
    matrix_row = rows // square_size
    matrix_col = cols // square_size

    for square_row in range(matrix_row):
        square_cols = [""] * square_size  

        for chunk_col in range(matrix_col):
            for i in range(square_size):
                line = " ".join(f"{int(matrix[square_row * square_size + i, chunk_col * square_size + j]):2}"
                                for j in range(square_size))
                square_cols[i] += line + "   " 
        
        for line in square_cols:
            print(line)
        print("\n" + "-" * 30)  

        
def gate_change(gates, last_gate):  # changes grid based on both gates and last gate
    # normal gates
    for i in range(len(gates)):
        grid[(gates[i-1][0]*3)+1, (gates[i-1][1]*3)+1] = 3
    #last gate
    grid[(last_gate[0]*3)+1, (last_gate[1]*3)+1] = 4


def start_change(start_point):  # changes grid based on start point
    # makes all the points on the outside 0

    # does top and bottom
    for col in range(12):
        grid[0, col] = -1
        grid[14, col] = -1
    # does sides
    for row in range(15):
        grid[row, 0] = -1
        grid[row, 11] = -1
    # adds the start point
    if (3 >= start_point) and (start_point >= 0):#top
        grid[0,(start_point*3)+1] = 1
    elif (start_point >= 4) and (start_point <= 8):#right side
        grid[((start_point-4)*3)+1,11] = 1
    elif (start_point >= 9) and (start_point <= 12):#bottom
        grid[14, ((start_point-9)*3)+1] = 1
    elif (start_point >= 13) and (start_point <= 17):#left side
        grid[((start_point-13)*3)+1, 0] = 1
    else:
        print('start_point invalid')


def block_change(blocks):  # changes grid based on blocks
    #table[row][col]
    for i in range(len(blocks)):
        if blocks[i-1][2] == 1:  # top
            grid[(blocks[i-1][0]*3), ((blocks[i-1][1]*3),(blocks[i-1][1]*3)+1,((blocks[i-1][1]*3)+2))] = -1
        elif blocks[i-1][2] == 2:  # right side
            grid[((blocks[i-1][0]*3),(blocks[i-1][0]*3)+1,(blocks[i-1][0]*3)+2),(blocks[i-1][1]*3)+2] = -1
        elif blocks[i-1][2] == 3:  # bottom
            grid[(blocks[i-1][0]*3)+2, ((blocks[i-1][1]*3),(blocks[i-1][1]*3)+1,((blocks[i-1][1]*3)+2))] = -1
        elif blocks[i-1][2] == 4:  # left side
            grid[((blocks[i-1][0]*3),(blocks[i-1][0]*3)+1,(blocks[i-1][0]*3)+2),blocks[i-1][1]*3] = -1
        else:
            print("invlid block or blocks")


# makes changes to the grid based on the points
def make_changes(start_point, end_point, gates, last_gate, blocks):
    # makes start change
    start_change(start_point)
    # makes end change
    grid[(end_point[0]*3)+1, (end_point[1]*3)+1] = 2
    #makes gate change
    gate_change(gates, last_gate)
    #make block change
    block_change(blocks)
    return grid

#makes distance matrix
def distance_matrix(start_point):
    table = np.full((15,12),np.inf)
    #adds the start point
    if (3 >= start_point) and (start_point >= 0):#top
        table[0,(start_point*3)+1] = 1
    elif (start_point >= 4) and (start_point <= 8):#right side
        table[((start_point-4)*3)+1,11] = 1
    elif (start_point >= 9) and (start_point <= 12):#bottom
        table[14, ((start_point-9)*3)+1] = 1
    elif (start_point >= 13) and (start_point <= 17):#left side
        table[((start_point-13)*3)+1, 0] = 1
    else:
        print('error in value of start_point')
