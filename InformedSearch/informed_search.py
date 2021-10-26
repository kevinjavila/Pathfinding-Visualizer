# State definition: Node is at location (x, y)

# Initial State: Wherever initial node is (assuming it exists in the plane)

# Actions: Possibility to translate up, down, left or right at each iteration

# Transition Model: Node completing action of translating up, down, left or right (as long as it
# is in bounds), while returning the new state

# Goal Test: Wherever initial node should be at the end of the search (assuming it exists in
# the plane)

# Path Cost: The sum of all actual numbers (not coordinates) that it went through to get to
# goal node


import heapq

class Node():
    def __init__(self, value, parent, g, h):
        self.value = value
        self.parent = parent
        self.g = g # path cost
        self.h = h # heuristic
        self.f = g + h # path cost + heuristic


    def __lt__(self, other):
        f1 = self.f
        f2 = other.f

        return f1 < f2

def heurisitc(location, goal):
    x = abs(location[0] - goal[0])
    y = abs(location[1] - goal[1])
    return (x + y)


def getNeighbors(location, grid):
    output = []
    rows = len(grid) - 1
    columns = len(grid[0]) - 1

    # All possible movements of location (left, right, up, down)
    translations = [[0, 1], [1, 0], [0, -1], [-1, 0]]

    # Checking all possible translations (left, right, up, down)
    for translation in translations:
        cord = [location[0] + translation[0], location[1] + translation[1]]
        # Checking if location exists
        if cord[0] > -1 and cord[0] <= columns:
            if cord[1] > -1 and cord[1] <= rows:
                # Checking if neighbor is greater than 0
                if grid[cord[0]][cord[1]] > 0:
                    output.append(cord)

    return output


def getValue(lst):
    # Getting values of objects
    values = []
    for node in lst:
        values.append(node.value)
    return values


def expandNode(node, goal_node, grid, closed_list, open_list):
    coords = node.value
    coords_goal = goal_node
    neighbors = getNeighbors(coords, grid)
    # Initializing temp lists
    open_list_values = getValue(open_list)
    closed_list_values = getValue(closed_list)
    # Checking if node is in open or closed list
    for position in neighbors:
        if position not in closed_list_values and position not in open_list_values:
            new_node = Node(position, node, node.g + grid[position[0]][position[1]], heurisitc(position, goal_node))
            heapq.heappush(open_list, new_node)


def informed_search(node, goal_node, grid, open_list, closed_list):
    initial_node = node
    open_list = [node]
    closed_list = []

    # Using a loop to make sure that the length of the open list is not 0
    while len(open_list) != 0:
        current = open_list.pop(0)
        closed_list.append(current)
        # Once the loop finds goal node, it will add up the path cost and # of expanded nodes
        if current.value == goal_node:
            expanded_nodes = len(closed_list)
            path_cost = 0
            path = []
            while current.value != initial_node.value:
                path_cost += grid[current.value[0]][current.value[1]]
                path.append(current)
                current = current.parent
            path.append(current)
            print("A* Found you {}. Your path cost was {}. Number of expanded nodes is {}.".format(goal_node, path_cost, expanded_nodes))

            return getValue(path)

            # If current node is not goal node, get new neighbors by using expandNode
        else:
            expandNode(current, goal_node, grid, closed_list, open_list)

def read_grid(filename):
    grid = []
    with open(filename) as f:
        for i in f.readlines():
            grid.append([int(x) for x in i.split()])

    f.close()
    return grid


def outputGrid(grid, start, goal, path):
    # print('In outputGrid')
    filenameStr = 'path.txt'

    # Open filename
    f = open(filenameStr, 'w')

    # Mark the start and goal points
    grid[start[0]][start[1]] = 'S'
    grid[goal[0]][goal[1]] = 'G'

    # Mark intermediate points with *
    for i, p in enumerate(path):
        if i > 0 and i < len(path) - 1:
            grid[p[0]][p[1]] = '*'

    # Write the grid to a file
    for r, row in enumerate(grid):
        for c, col in enumerate(row):

            # Don't add a ' ' at the end of a line
            if c < len(row) - 1:
                f.write(str(col) + ' ')
            else:
                f.write(str(col))

        # Don't add a '\n' after the last line
        if r < len(grid) - 1:
            f.write("\n")

    # Close file
    f.close()
    # print('Exiting outputGrid')


def main():
    goal_node = [0, 0] # does not have to be node object
    initial_node = Node([9, 9], None, 0, heurisitc([3, 6], goal_node))

    grid = read_grid("grid.txt")

    closed_list = []
    open_list = []
    heapq.heapify(open_list)

    path = informed_search(initial_node, goal_node, grid, open_list, closed_list)

    outputGrid(grid, [3, 6], goal_node, path)


main()
