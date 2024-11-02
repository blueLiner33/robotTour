#imports
import heaper as heap
import grid_maker as gm
def dijkstra(grid, start, goal):
    rows, cols = grid.shape
    priority_queue = []
    distances = {start: 0}
    previous_nodes = {start: None}

    heap.heappush(priority_queue, (0, start))

    directions = [(-3, 0), (3, 0), (0, -3), (0, 3)]  # Moving three points at a time

    while priority_queue:
        current_distance, current_node = heap.heappop(priority_queue)

        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = previous_nodes[current_node]
            return path[::-1]

        if current_distance > distances.get(current_node, float('inf')):
            continue

        for direction in directions:
            neighbor = (current_node[0] + direction[0], current_node[1] + direction[1])

            # Ensure the movement path is clear and within bounds
            if (
                0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and 
                grid[neighbor[0], neighbor[1]] != -1 and
                grid[(2 * current_node[0] + neighbor[0]) // 3, (2 * current_node[1] + neighbor[1]) // 3] != -1 and  # First midpoint
                grid[(current_node[0] + 2 * neighbor[0]) // 3, (current_node[1] + 2 * neighbor[1]) // 3] != -1  # Second midpoint
            ):
                distance = current_distance + 1

                if distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heap.heappush(priority_queue, (distance, neighbor))

    raise TypeError('check points entered')

def closest_point(grid,start,goals):#finds the closest point from a tuple 
    shortest = []
    for goal in goals:
        length = dijkstra(grid,start,goal)
        shortest.append((length,goal))
    shortest.sort()
    return shortest[0][1]

def pathing(grid,gates,start_point,last_gate,final_point):
    ungotten_gates = gates
    commands = []
    last_point = start_point
    for i in ungotten_gates:
        i*=1#just want it green
        current_goal = closest_point(grid,start_point,ungotten_gates)
        for command in (dijkstra(grid, last_point, current_goal)):
            commands.append(command)
        ungotten_gates.remove(current_goal)
        last_point = current_goal
    grid[last_gate[0], last_gate[1]] = 4
    for command in (dijkstra(grid, last_point, last_gate)):
        commands.append(command)
    last_point = last_gate
    for command in (dijkstra(grid, last_point,final_point )):
        commands.append(command)
    return commands

def list_commands(commands,start_point):
    heading = None
    if (3 >= start_point) and (start_point >= 0):#top
        heading = 'down'
    elif (start_point >= 4) and (start_point <= 8):#right side
        heading = 'left'
    elif (start_point >= 9) and (start_point <= 12):#bottom
        heading = 'up'
    elif (start_point >= 13) and (start_point <= 17):#left side
        heading = 'right'
    #180 = 3,forward = 0, left = 1,right =2
    prior_point = commands[0]
    sequence_commands = []
    for element in commands:
        if prior_point[0] < element[0]:#right
            if heading == 'down':
                sequence_commands.append(1)
                sequence_commands.append(0)
                heading = 'right'
            elif heading == 'left':
                sequence_commands.append(3)
                sequence_commands.append(0)
                heading = 'right'
            elif heading == 'up':
                sequence_commands.append(2)
                sequence_commands.append(0)
                heading = 'right'
            elif heading == 'right':
                sequence_commands.append(0)
            prior_point = element
        elif prior_point[0] > element[0]:#left
            if heading == 'down':
                sequence_commands.append(2)
                sequence_commands.append(0)
                heading = 'left'
            elif heading == 'left':
                sequence_commands.append(0)
            elif heading == 'up':
                sequence_commands.append(1)
                sequence_commands.append(0)
                heading = 'left'
            elif heading == 'right':
                sequence_commands.append(3)
                sequence_commands.append(0)
                heading = 'left'
            prior_point = element
        elif prior_point[1]<element[1]:#up
            if heading == 'down':
                sequence_commands.append(3)
                sequence_commands.append(0)
                heading = 'up'
            elif heading == 'left':
                sequence_commands.append(2)
                sequence_commands.append(0)
                heading = 'up'
            elif heading == 'up':
                sequence_commands.append(0)
            elif heading == 'right':
                sequence_commands.append(1)
                sequence_commands.append(0)
                heading = 'up'
            prior_point = element
        elif prior_point[1]>element[1]:#down
            if heading == 'down':
                sequence_commands.append(0)
            elif heading == 'left':
                sequence_commands.append(1)
                sequence_commands.append(0)
                heading = 'down'
            elif heading == 'up':
                sequence_commands.append(3)
                sequence_commands.append(0)
                heading = 'down'
            elif heading == 'right':
                sequence_commands.append(2)
                sequence_commands.append(0)
                heading = 'down'
            prior_point = element
    return sequence_commands

def give_commands(start_point,end_point,gate_points,last_gate_point,blocks):
    grid = gm.make_changes(start_point,end_point,gate_points,last_gate_point,blocks)
    first_point,end_spot,final_gate,goal_points = gm.get_cords(start_point,end_point,last_gate_point,gate_points)
    commands = pathing(grid,goal_points,first_point,final_gate,end_spot)
    return list_commands(commands,start_point)