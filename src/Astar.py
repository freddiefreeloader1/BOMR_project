import heapq
import timeit

import matplotlib.pyplot as plt

import numpy as np
import random


moves_8n = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

moves_4n = [(0, -1), (0, 1), (-1, 0), (1, 0)]



def plot_path(maze, path, start, end):

    maze_with_path = [row.copy() for row in maze]


    for position in path:

        maze_with_path[position[1]][position[0]] = 2  # Marking the path


    maze_with_path[start[1]][start[0]] = 3  

    maze_with_path[end[1]][end[0]] = 4  


    maze_array = np.array(maze_with_path, dtype=np.int32)


    cmap = plt.cm.colors.ListedColormap(['white', 'red', 'green', 'blue', 'purple'])

    bounds = [0, 1, 2, 3, 4, 5]

    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    plt.imshow(maze_array, cmap=cmap, norm=norm, origin='upper', interpolation='none')


    plt.xticks(ticks=np.arange(0, len(maze[0]), 1)-0.5, labels=[])

    plt.yticks(ticks=np.arange(0, len(maze), 1)-0.5,labels=[])


    plt.xlabel('X-axis')

    plt.ylabel('Y-axis')


    plt.scatter(0, 0, marker='o', color='black', label='(0,0)')


    plt.scatter(path[0][0], path[0][1], marker='o', color='cyan', label='Start')

    plt.scatter(path[-1][0], path[-1][1], marker='o', color='magenta', label='End')


    plt.grid(True, color='black', linewidth=0.5)


    plt.show()



class Node:

    def __init__(self, parent=None, position=None):

        self.parent = parent
        self.position = position

        self.cost_of_move = 0

        self.heuristic = 0
        self.total_cost = 0


    def __eq__(self, other):

        return self.position == other.position


    def __lt__(self, other):

        if self.total_cost == other.total_cost:

            return id(self) < id(other)

        return self.total_cost < other.total_cost



def astar_grid(maze, start, end, moves):
    
    if maze[start[0]][start[1]] == 1:
        print("Can't start from here!")
        return None 

    start_node = Node(None, start)

    start_node.cost_of_move = start_node.heuristic = start_node.total_cost = 0


    end_node = Node(None, end)

    end_node.cost_of_move = end_node.heuristic = end_node.total_cost = 0

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    while open_list:


        current_node = heapq.heappop(open_list)


        closed_set.add(current_node.position)


        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.position)

                current_node = current_node.parent

            return path[::-1]

        children = [

            Node(current_node, (current_node.position[0] + dx, current_node.position[1] + dy))

            for dx,dy in moves
            if (

                0 <= current_node.position[0] + dx < len(maze[0])

                and 0 <= current_node.position[1] + dy < len(maze)

                and (maze[current_node.position[1] + dy][current_node.position[0] + dx] == 0 

                or maze[current_node.position[1] + dy][current_node.position[0] + dx] == "E")

                and (current_node.position[0] + dx, current_node.position[1] + dy) not in closed_set
            )
        ]


        for child in children:

            dx, dy = child.position[0] - current_node.position[0], child.position[1] - current_node.position[1]

            child.cost_of_move = current_node.cost_of_move + 1.41 if dx != 0 and dy != 0 else current_node.cost_of_move + 1

            child.heuristic = ((child.position[0] - end_node.position[0]) ** 2

                            + (child.position[1] - end_node.position[1]) ** 2)

            child.total_cost = child.cost_of_move + child.heuristic

            heapq.heappush(open_list, child)


def find_start_end(grid):

    for i in range(len(grid)):

        for j in range(len(grid[0])):

            if grid[i][j] == "S":

                Start = (j,i)

            if grid[i][j] == "E":

                End = (j,i)

    return Start, End


def generate_random_maze(rows, cols, start, end, obstacle_prob):

    maze = [["0" for _ in range(cols)] for _ in range(rows)]


    maze[start[1]][start[0]] = "S"

    maze[end[1]][end[0]] = "E"


    for row in range(rows):

        for col in range(cols):

            if maze[row][col] == "S" or maze[row][col] == "E":

                continue


            if random.uniform(0, 1) < obstacle_prob:

                maze[row][col] = 1

            else:

                maze[row][col] = 0


    return maze


def print_maze(maze):

    for row in maze:

        print(" ".join(map(str, row)))



def search():


    rows = 50

    cols = 50

    start_point = (random.randint(0, cols - 1), random.randint(0, rows - 1))

    end_point = (random.randint(0, cols - 1), random.randint(0, rows - 1))

    obstacle_probability = 0.2


    random_maze = generate_random_maze(rows, cols, start_point, end_point, obstacle_probability)


    start, end = find_start_end(random_maze)

    print(start, end)

    # plot_path(random_maze,[start,end],start,end)
    

    astar_time = timeit.timeit(lambda: astar_grid(random_maze, start, end, moves_8n), number=1)


    print(f"Total running time: {astar_time * 10**3:.3f} ms")

    path = astar_grid(random_maze, start, end, moves_8n)


    if path:

        # print("Path found:", path)

        plot_path(random_maze, path, start, end)

    else:

        print("No path found.")


if __name__ == "__main__":
    search()