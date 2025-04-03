# Import any libraries required
import random


# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
# DO NOT EDIT THIS FUNCTION DECLARATION


def do_a_star(grid, start, end, display_message):

    """
    Perform A* pathfinding on a given grid.

    Args:
        grid (list of list): 2D grid where 1 is walkable, 0 is an obstacle.
        start (tuple): (col, row) start position.
        end (tuple): (col, row) goal position.
        display_message (function): Function to display messages in GUI.

    Returns:
        list: Shortest path from start to end as a list of (col, row).
              Returns an empty list if no path is found.

    """


    # Get the size of the grid
    COL = len(grid)      # Number of columns
    ROW = len(grid[0])   # Number of rows
    
    # Allowed movements (up, down, left, right)
    moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    
    # Heuristic function (Euclidean distance) (h(n))
    def heuristic(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
    
    # This list stores tuples of (f(n), (col, row))
    open_set = [(0, start)]  
    
    # Dictionaries to store path and cost
    came_from = {}  # Store path
    g_score = {start: 0}  # Cost from start to each node (g(n)), g(n) is Manhattan distance from start point
    
    closed_set = set()  # Set to store visited nodes

    while open_set:
        open_set.sort()  # Sort open_set to find the node with the lowest f(n)
        _, current = open_set.pop(0)  # Get node with lowest f(n)

        # Add the node to the closed set to avoid re-processing
        closed_set.add(current)

        if current == end:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()

            display_message("Path found! Path: " + str(path))
            display_message("End location is " + str(end))
            display_message("Start location is " + str(start))
            return path  # Send the path back to the GUI to be displayed
        
        for move in moves:
            neighbor = (current[0] + move[0], current[1] + move[1])  # Possible move
            
            # Check bounds and obstacles
            if not (0 <= neighbor[0] < COL and 0 <= neighbor[1] < ROW):
                continue
            if grid[neighbor[0]][neighbor[1]] == 0:  # 0 means obstacle
                continue
            if neighbor in closed_set:  # Skip already processed nodes
                continue

            new_g_score = g_score[current] + 1  # Update g(n)
            
            
            ####################################################################
            # Core A* Algorithm:
            # g(n) is Manhattan distance from the start point to the current point
            # h(n) is Euclidean distance from the current point to the end point
            # f(n) = g(n) + h(n) is the estimated total cost
            ####################################################################

            if neighbor not in g_score or new_g_score < g_score[neighbor]:
                g_score[neighbor] = new_g_score
                f_score = new_g_score + heuristic(neighbor, end)
                open_set.append((f_score, neighbor))
                came_from[neighbor] = current
    
    display_message("No path found")
    return []  # No path found

# end of file
