import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys
import math
from heapq import heappush, heappop

# Define moves and their corresponding angles
moves = {
    0: (1, 0, 0),       # East
    1: (1, 1, 45),      # Northeast
    2: (0, 1, 90),      # North
    3: (-1, 1, 135),    # Northwest
    4: (-1, 0, 180),    # West
    5: (-1, -1, 225),   # Southwest
    6: (0, -1, 270),    # South
    7: (1, -1, 315)     # Southeast
}

def read_input(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    # Read the first line to get start and goal positions
    first_line = lines[0].strip()
    tokens = first_line.split()
    start_i, start_j, goal_i, goal_j = map(int, tokens)

    start_pos = (start_i, start_j)
    goal_pos = (goal_i, goal_j)

    # Initialize maze
    maze = np.zeros((30, 50), dtype=int)

    # Read the cell values of the robot workspace
    for line_num in range(1, 31):
        line = lines[line_num].strip()
        tokens = line.split()
        if len(tokens) < 50:
            print(f"Error: Expected 50 values in line {line_num + 1}, got {len(tokens)}")
            sys.exit(1)
        maze_row = list(map(int, tokens))
        j = 31 - (line_num + 1)
        # Store each row in maze
        maze[j, :] = maze_row

    return start_pos, goal_pos, maze

def euclidean_distance(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]

    return math.hypot(dx, dy)

class Node:
    def __init__(self, position, g, h, f, parent, action, theta):
        self.position = position  # (i, j)
        self.g = g  # cost from start to this node
        self.h = h  # heuristic cost to goal
        self.f = f  # total cost
        self.parent = parent  # parent node
        self.action = action  # action taken to reach this node
        self.theta = theta  # angle at this node

    def __lt__(self, other):
        return self.f < other.f
        
def Astar_search(start_pos, goal_pos, maze, k):
    frontier = [] # priority queue
    expanded_nodes = set()
    reached = {} # lookup table 

    # Initialize root node
    root = Node(position=start_pos, g=0, h=0, f=0, parent=None, action=None, theta=None)
    root.h = euclidean_distance(start_pos, goal_pos)
    root.f = root.h
    heappush(frontier, (root.f, root))

    nodes_generated = 1  # root node

    while frontier:
        # Pop the node with the lowest f(n)
        current_f, current_node = heappop(frontier)
        current_position = current_node.position

        # Skip if this position is already expanded
        if current_position in expanded_nodes:
            continue

        # Check if it is the goal node
        if current_position == goal_pos:
            # Solution path
            path = []
            actions = []
            f_values = []
            node = current_node
            while node is not None:
                path.append(node.position)
                f_values.append(node.f)
                if node.action is not None:
                    actions.append(node.action)
                node = node.parent
            path.reverse()
            f_values.reverse()
            actions.reverse()
            depth = len(path) - 1  
            return depth, nodes_generated, actions, f_values, path

        # Expand node
        expanded_nodes.add(current_position)

        for action, (dx, dy, theta2) in moves.items():
            neighbor_i = current_position[0] + dx
            neighbor_j = current_position[1] + dy
            neighbor_position = (neighbor_i, neighbor_j)

            # Check if neighbor node is within maze bound:
            if 0 <= neighbor_i < 50 and 0 <= neighbor_j < 30:
                # Check if neighbor node is not an obstacle and not in expanded_nodes set
                if maze[neighbor_j][neighbor_i] != 1 and neighbor_position not in expanded_nodes:
                    # Compute costs
                    # Distance cost
                    if action in [0, 2, 4, 6]:
                        c_d = 1
                    else:
                        c_d = math.sqrt(2)
                    # Angle cost
                    if current_node.theta is None: # start position
                        c_a = 0
                    else:
                        d_theta = abs(theta2 - current_node.theta)
                        if d_theta > 180:
                            d_theta = 360 - d_theta
                        c_a = k * (d_theta / 180)
                    c = c_d + c_a
                    g = current_node.g + c
                    h = euclidean_distance(neighbor_position, goal_pos)
                    f = g + h

                    # Check if neighbor node is already generated with a lower f value
                    if neighbor_position in reached:
                        existing_node = reached[neighbor_position]
                        if f < existing_node.f:
                            existing_node.g = g
                            existing_node.h = h
                            existing_node.f = f
                            existing_node.parent = current_node
                            existing_node.action = action
                            existing_node.theta = theta2
                            # Push the updated node back into the queue
                            heappush(frontier, (existing_node.f, existing_node))
                    else:
                        neighbor_node = Node(position=neighbor_position, g=g, h=h, f=f, parent=current_node, action=action, theta=theta2)
                        heappush(frontier, (f, neighbor_node))
                        reached[neighbor_position] = neighbor_node
                        nodes_generated += 1
    
    # No path found -- failure
    return None, None, None, None, None

def write_output(output_file, depth, nodes_generated, actions, f_values, path, maze):
    with open(output_file, 'w') as f:
        f.write(f"{depth}\n")
        f.write(f"{nodes_generated}\n")
        f.write(' '.join(map(str, actions)) + '\n')
        f.write(' '.join(['{:.2f}'.format(val) for val in f_values]) + '\n')

        # robot workspace
        # 0: while; 1: black; 2: start pos; 5: goal pos; 4: solution path
        new_maze = np.copy(maze)
        start_i, start_j = path[0]
        goal_i, goal_j = path[-1]
        new_maze[start_j][start_i] = 2
        new_maze[goal_j][goal_i] = 5
        for pos in path[1:-1]:
            i, j = pos
            new_maze[j][i] = 4
        
        for j in range(29, -1, -1):
            line = new_maze[j, :]
            line_str = ' '.join(map(str, line))
            f.write(line_str + '\n')


def plot_maze(file_path):
    try:
        # Load the maze from the .txt file
        with open(file_path, "r") as file:
            lines = file.readlines()

        maze_lines = lines[4:34]
        maze = []
        for line in maze_lines:
            tokens = line.strip().split()
            maze_row = list(map(int, tokens))
            if len(maze_row) != 50:
                raise ValueError("Each maze row must contain 50 values.")
            maze.append(maze_row)

        maze = np.array(maze)

        # Initialize the plot
        fig, ax = plt.subplots(figsize=(12, 7))
        nrows, ncols = maze.shape

        # Define colors for each tile type
        colors = {0: "white", 1: "black", 2: "red", 3: "white", 4: "yellow", 5: "green"}

        # Plot each tile
        for row in range(nrows):
            for col in range(ncols):
                tile_color = colors.get(
                    maze[row, col], "white"
                )  # Default to white if unknown
                rect = Rectangle(
                    (col, row), 1, 1, edgecolor="black", facecolor=tile_color
                )
                ax.add_patch(rect)

        plt.xlim(0, ncols)
        plt.ylim(0, nrows)
        plt.gca().invert_yaxis()  # Invert y-axis to match array layout
        plt.axis("off")  # Turn off the axis
        plt.show()

    except Exception as e:
        print(f"Failed to read the file: {e}")


def main():
    if len(sys.argv) != 4:
        print("Usage: python script_name.py input_file output_file k_value")
        sys.exit(1)

    file_path = sys.argv[1]
    output_file = sys.argv[2]
    k = float(sys.argv[3])
    start_pos, goal_pos, maze = read_input(file_path)
    depth, nodes_generated, actions, f_values, path = Astar_search(start_pos, goal_pos, maze, k)
    if depth is not None:
        write_output(output_file, depth, nodes_generated, actions, f_values, path, maze)
        print("Path found. Output written to", output_file)
    else:
        print("No path found")
    plot_maze(output_file)



if __name__ == "__main__":
    main()