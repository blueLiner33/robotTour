import functools
#module only for debugging other sections of code

def debug(func):#returns arguments of function and valeu
    @functools.wraps(func)
    def wrapper_debug(*args, **kwargs):
        args_repr = [repr(a) for a in args]
        kwargs_repr = [f"{k}={repr(v)}" for k, v in kwargs.items()]
        signature = ", ".join(args_repr + kwargs_repr)
        print(f"Calling {func.__name__}({signature})")
        value = func(*args, **kwargs)
        print(f"{func.__name__}() returned {repr(value)}")
        return value
    return wrapper_debug

def print_matrix(matrix):  #print matrix useful for testing if path solver is working right
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
