# Custom function to create a matrix of zeros
def zeros(rows, cols):
    return [[0 for _ in range(cols)] for _ in range(rows)]

grid = zeros(15, 12)  # makes a 15x12 matrix filled with zeros

def print_matrix(matrix):  # testing function
    square_size = 3  
    rows, cols = len(matrix), len(matrix[0])
    matrix_row = rows // square_size
    matrix_col = cols // square_size

    for square_row in range(matrix_row):
        square_cols = [""] * square_size  

        for chunk_col in range(matrix_col):
            for i in range(square_size):
                line = " ".join(f"{int(matrix[square_row * square_size + i][chunk_col * square_size + j]):2}"
                                for j in range(square_size))
                square_cols[i] += line + "   " 
        
        for line in square_cols:
            print(line)
        print("\n" + "-" * 30)


def gate_change(gates, last_gate):  # changes grid based on both gates and last gate
    # normal gates
    for i in range(len(gates)):
        grid[(gates[i][0]*3)+1][(gates[i][1]*3)+1] = 3
    # last gate
    grid[(last_gate[0]*3)+1][(last_gate[1]*3)+1] = -1

def start_change(start_point):  # changes grid based on start point
    # makes all the points on the outside 0
    # does top and bottom
    for col in range(12):
        grid[0][col] = -1
        grid[14][col] = -1
    # does sides
    for row in range(15):
        grid[row][0] = -1
        grid[row][11] = -1
    # adds the start point
    if (3 >= start_point) and (start_point >= 0):  # top
        grid[1][(start_point*3)+1] = 1
    elif (start_point >= 4) and (start_point <= 8):  # right side
        grid[((start_point-4)*3)+1][10] = 1
    elif (start_point >= 9) and (start_point <= 12):  # bottom
        grid[13][((start_point-9)*3)+1] = 1
    elif (start_point >= 13) and (start_point <= 17):  # left side
        grid[((start_point-13)*3)+1][1] = 1
    else:
        raise ValueError('start_point invalid')

def get_cords(begin_point, stop_point, final_gate, gate_points):  # returns the cords in the right format
    gate_cords = []  # gate cord variable
    last_cord = []  # last cord variable
    for i in range(len(gate_points)):  # goes all the gates
        gate_cords.append(((gate_points[i][0]*3)+1, (gate_points[i][1]*3)+1))  # adds them
    last_cord = (final_gate[0]*3)+1, (final_gate[1]*3)+1
    # end and start
    end = (stop_point[0]*3)+1, (stop_point[1]*3)+1  # stores end
    start = None  # start variable
    if (3 >= begin_point) and (begin_point >= 0):  # top
        start = (1, (begin_point*3)+1)
    elif (begin_point >= 4) and (begin_point <= 8):  # right side
        start = (((begin_point-4)*3)+1, 10)
    elif (begin_point >= 9) and (begin_point <= 12):  # bottom
        start = (13, ((begin_point-9)*3)+1)
    elif (begin_point >= 13) and (begin_point <= 17):  # left side
        start = (((begin_point-13)*3)+1, 1)

    return start, end, last_cord, gate_cords

def block_change(blocks):  # changes grid based on blocks
    #table[row][col]
    for i in range(len(blocks)):
        # Correct indexing for blocks, checking each side for the proper changes
        row, col, side = blocks[i]
        
        if side == 1:  # top
            for j in range(3):
                grid[row*3][col*3 + j] = -1
        elif side == 2:  # right side
            for j in range(3):
                grid[row*3 + j][col*3 + 2] = -1
        elif side == 3:  # bottom
            for j in range(3):
                grid[row*3 + 2][col*3 + j] = -1
        elif side == 4:  # left side
            for j in range(3):
                grid[row*3 + j][col*3] = -1
        else:
            print("Invalid block or side")

# makes changes to the grid based on the points
def make_changes(start_point, end_point, gates, last_gate, blocks):
    # makes end change
    grid[(end_point[0]*3)+1][(end_point[1]*3)+1] = 2
    # makes gate change
    gate_change(gates, last_gate)
    # makes block change
    block_change(blocks)
    # makes start change
    start_change(start_point)
    return grid
