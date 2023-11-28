import heapq
import timeit
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
import random

def plot_path(coords, path):
    
    codes = [Path.MOVETO] + [Path.LINETO] * (len(path) - 1)

    path_obj = Path(path, codes)

    fig, ax = plt.subplots()
    
    patch = patches.PathPatch(path_obj, facecolor='none', lw=2)
    ax.add_patch(patch)
    
    xs_path, ys_path = zip(*path)
    ax.plot(xs_path, ys_path, 'x--', lw=2, color='black', ms=10, label='On Path')
    
    xs_not_on_path, ys_not_on_path = zip(*[coord for coord in coords if coord not in path])
    ax.scatter(xs_not_on_path, ys_not_on_path, marker='o', color='red', label='Not on Path')
    
    for i, coord in enumerate(coords):
        label = f'P{i}'
        ax.text(coord[0], coord[1], label)
    
    ax.legend()
    
    # Set limits and show plot
    ax.set_xlim(min(xs_path + xs_not_on_path) - 1, max(xs_path + xs_not_on_path) + 1)
    ax.set_ylim(min(ys_path + ys_not_on_path) - 1, max(ys_path + ys_not_on_path) + 1)
    
    plt.show()


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.cost_of_move = 0.0
        self.heuristic = 0.0
        self.total_cost = 0.0

    def __eq__(self, other):
        return math.isclose(self.position[0], other.position[0]) and math.isclose(
            self.position[1], other.position[1])

    def __lt__(self, other):
        if math.isclose(self.total_cost, other.total_cost):
            return id(self) < id(other)
        return self.total_cost < other.total_cost

def euclidean_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

def find_closest_neighbors(coord, coords, num_neighbors):
    return heapq.nsmallest(num_neighbors, coords, key=lambda x: euclidean_distance(coord, x))

def neighbors_in_circle(center, coords, diameter):
    return [coord for coord in coords if euclidean_distance(center, coord) <= diameter]

def astar(coords, start, end, method):
    start_node = Node(None, start)
    start_node.cost_of_move = start_node.heuristic = start_node.total_cost = 0.0

    end_node = Node(None, end)
    end_node.cost_of_move = end_node.heuristic = end_node.total_cost = 0.0

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

        neighbors = [
            (x, y)
            for x, y in coords
            if (x, y) not in closed_set and (x, y) != current_node.position
        ]

        if method == "circle":
            closest_neighbors = neighbors_in_circle(current_node.position, neighbors, 3)
        elif method == "closest_n":
            closest_neighbors = find_closest_neighbors(current_node.position, neighbors, 7)

        if closest_neighbors == []:
            print("Increase the diameter value")
            break

        children = [
            Node(current_node, neighbor)
            for neighbor in closest_neighbors
        ]

        for child in children:
            child.cost_of_move = (
                current_node.cost_of_move + euclidean_distance(current_node.position, child.position)
            )
            child.heuristic = euclidean_distance(child.position, end_node.position)
            child.total_cost = child.cost_of_move + child.heuristic
            heapq.heappush(open_list, child)


def create_random_coordinates(num_coordinates,max_lim):
    coordinates = [(random.uniform(0, max_lim), random.uniform(0, max_lim)) for _ in range(num_coordinates)]
    return coordinates

def search():

    coord = create_random_coordinates(20,2000)
    print(coord)
    start = random.choice(coord)
    end = heapq.nlargest(1,coord,key= lambda x: euclidean_distance(start,x))[0]
    
    # astar_time_circle = timeit.timeit(lambda: astar(coord, start, end,"circle"), number=1)
    astar_time_closest_n = timeit.timeit(lambda: astar(coord, start, end,"circle"), number=1)
    #print(f"Total running time for circle method: {astar_time_circle * 10**3:.3f} ms")
    print(f"Total running time for closest n method: {astar_time_closest_n * 10**3:.3f} ms")

    # path1 = astar(coord, start, end, "circle")
    path2 = astar(coord, start, end, "closest_n")
    if path2:
        '''print("Path found:", path1)
        print("Path found:", path2)'''
        # plot_path(coord, path1)
        plot_path(coord, path2)
    else:
        print("No path found.")

if __name__ == "__main__":
    search()
