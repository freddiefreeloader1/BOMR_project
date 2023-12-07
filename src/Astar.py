# imports
import heapq
import timeit
import cv2
import matplotlib.pyplot as plt
from computer_vision import * 
import numpy as np
import random

# possible moves from one square to another
moves_8n = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

moves_4n = [(0, -1), (0, 1), (-1, 0), (1, 0)]



class Node:
    """
    Represents a node in the A* algorithm.

    Attributes:
    - parent (Node): The parent node in the search tree.
    - position (Tuple[int, int]): The position (coordinates) of the node on the grid.
    - cost_of_move (float): The cost of reaching this node from the start.
    - heuristic (float): The estimated cost to reach the goal from this node.
    - total_cost (float): The sum of the cost_of_move and heuristic.

    Methods:
    - __eq__(self, other): Compares two nodes based on their positions.
    - __lt__(self, other): Compares two nodes based on their total_cost 
    
    """
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
    """
    A* search algorithm for finding the shortest path on a 2D grid.

    Parameters:
    - maze (numpy.ndarray): A 2D grid where 0 represents an open path, and 1 represents an obstacle.
    - start (Tuple[int, int]): The starting point on the grid.
    - end (Tuple[int, int]): The destination point on the grid.
    - moves (List[Tuple[int, int]]): A list of tuples representing possible moves from a given position.

    Returns:
    - np.array[Tuple[int, int]] or None: The shortest path from the start to the end on the grid, or None if no path is found.

    Node Class:
    - The algorithm uses a Node class to represent a point in the search space, which includes information about the node's position,
      cost of movement from the start, heuristic (estimated cost to reach the goal), and the total cost.

    Assumptions:
    - The maze is represented as a NumPy array where 0 denotes an open path, and 1 denotes an obstacle.

    Example:
    >>> maze = np.array([[0, 0, 0, 1, 0],
    ...                  [0, 1, 0, 1, 0],
    ...                  [0, 1, 0, 0, 0],
    ...                  [0, 0, 0, 1, 0],
    ...                  [0, 0, 0, 0, 0]])
    >>> start = (0, 0)
    >>> end = (4, 4)
    >>> moves_8n = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (-1, 1), (1, -1)]
    >>> path = astar_grid(maze, start, end, moves_8n)
    """

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
           
            child.heuristic = abs(child.position[0] - end_node.position[0]) \

            + abs(child.position[1] - end_node.position[1])

            child.total_cost = child.cost_of_move + child.heuristic

            heapq.heappush(open_list, child)



def simplify_path(path):

    """
    Simplifies a path by removing unnecessary intermediate points.

    Parameters:
    - path (np.array[Tuple[int, int]]): The original path represented as a list of coordinates.

    Returns:
    - np.array[Tuple[int, int]]: The simplified path with unnecessary intermediate points removed.
    
    Example:
    >>> simplify_path([(0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2)])
    [(0, 0), (3, 0), (3, 2)]

    """
    simplified_path = [path[0]] 

    for i in range(1, len(path) - 1):
        current_point = np.array(path[i])
        next_point = np.array(path[i + 1])
        direction_vector = next_point - np.array(simplified_path[-1])
        
        if np.cross(direction_vector, current_point - np.array(simplified_path[-1])) == 0:
            continue  

        simplified_path.append(path[i]) 

    simplified_path.append(path[-1])

    return simplified_path

def make_path(map_img, obstacle_masks, cell_size, start, end, grid, map_x = 600, map_y = 400):
    """
    Generates a path on a grid-based map using A* algorithm, considering obstacles on the image.

    Parameters:
    - map_img (numpy.ndarray): The original map image.
    - obstacle_masks (List[numpy.ndarray]): List of obstacle masks on the map.
    - cell_size (int): Size of each grid cell.
    - start (Tuple[int, int]): Starting point in image coordinates (x, y).
    - end (Tuple[int, int]): Ending point in image coordinates (x, y).
    - grid (numpy.ndarray): The grid representing the map.
    - map_x (int): Width of the map in image coordinates.
    - map_y (int): Height of the map in image coordinates.

    Returns:
    - Tuple[numpy.ndarray, List[Tuple[int, int]], List[Tuple[int, int]], List[Tuple[float, float]]]:
        - grid (numpy.ndarray): The grid with obstacle information.
        - path_grid (List[Tuple[int, int]]): The path on the grid coordinates.
        - simplified_path (List[Tuple[int, int]]): The simplified path on the grid coordinates.
        - metric_path (List[Tuple[float, float]]): The path in metric (image) coordinates.

    Example:
    >>> map_img = cv2.imread('map_image.png')
    >>> obstacle_masks = [obstacle_mask_1, obstacle_mask_2]
    >>> cell_size = 10
    >>> start = (100, 50)
    >>> end = (300, 200)
    >>> grid, path_grid, simplified_path, metric_path = make_path(map_img, obstacle_masks, cell_size, start, end, grid)
    """
    bw_map = cv2.cvtColor(map_img.copy(), cv2.COLOR_BGR2GRAY)
    grid = create_grid(bw_map, obstacle_masks, cell_size)
    start_grid = (grid.shape[1] * start[0] // map_img.shape[1], grid.shape[0] * start[1] // map_img.shape[0])
    end_grid = (grid.shape[1] * end[0] // map_img.shape[1], grid.shape[0] * end[1] // map_img.shape[0])
    path_grid = astar_grid(grid, start_grid, end_grid, moves_8n)
    if path_grid is None:
        print("no path found")
        return grid, None, None, None
    simplified_path = simplify_path(path_grid)
    metric_path = transform_grid_to_metric(simplified_path, map_x, map_y, grid)

    return grid, path_grid, simplified_path, metric_path


def transform_grid_to_metric(path, map_width, map_height, grid):
    """
    Transforms a path from grid coordinates to metric (image) coordinates.

    Parameters:
    - path (List[Tuple[int, int]]): The path in grid coordinates.
    - map_width (int): Width of the map in metric (image) coordinates.
    - map_height (int): Height of the map in metric (image) coordinates.
    - grid (numpy.ndarray): The grid representing the map.

    Returns:
    - List[Tuple[float, float]]: The path in metric (image) coordinates.

    Example:
    >>> path = [(0, 0), (3, 0), (3, 2)]
    >>> map_width = 600
    >>> map_height = 400
    >>> grid = np.zeros((4, 4), dtype=int)
    >>> metric_path = transform_grid_to_metric(path, map_width, map_height, grid)
    """
    metric_path = []
    grid_x = grid.shape[1]
    grid_y = grid.shape[0]
    for x,y in path:
        metric_path.append(((x * map_width) / grid_x, (y * map_height) / grid_y))

    return metric_path


if __name__ == "__main__":
    search()