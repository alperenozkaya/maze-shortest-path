import heapq

from pyamaze import maze, agent, textLabel
from queue import PriorityQueue
from collections import deque
import random
import time


def bfs(my_maze):
    """
    Breadth-First-Search algorithm to find the shortest path.
    :param
    my_maze: a randomly created, or loaded maze using pyamaze
    :return
    search_path: stores each iterated cell
    bfs_path: stores the shortest path from destination to start
    shortest_path: stores the shortest path obtained by traversing reversed_path
    """
    # initialize the starting cell as the bottom right corner, frontier queue,
    # path dictionary, and lists.
    start = (my_maze.rows, my_maze.cols)
    frontier = deque()
    frontier.append(start)
    bfs_path = {}  # path from destination to start
    explored = [start]
    search_path = []
    # keep searching until the frontier queue is empty
    while len(frontier) > 0:
        current_cell = frontier.popleft()  # next cell (first added element)
        if current_cell == my_maze._goal:  # check whether the cell is the destination
            break
        # check for the possible moves (East, West, North, South),
        # adjust the coordinates of the children cells on the possible move.
        for i in 'ESNW':
            if my_maze.maze_map[current_cell][i]:
                if i == 'E':
                    child_cell = (current_cell[0], current_cell[1] + 1)
                elif i == 'W':
                    child_cell = (current_cell[0], current_cell[1] - 1)
                elif i == 'N':
                    child_cell = (current_cell[0] - 1, current_cell[1])
                elif i == 'S':
                    child_cell = (current_cell[0] + 1, current_cell[1])
                # check whether the child cell is already explored.
                if child_cell in explored:
                    continue
                # add the child cell to frontier queue and explored list.
                frontier.append(child_cell)
                explored.append(child_cell)
                # add the child cell to the path dictionary, with the current cell as the key.
                bfs_path[child_cell] = current_cell
                search_path.append(child_cell)

    # create the shortest path by tracing back from the destination to the start.
    shortest_path = {}
    cell = my_maze._goal
    # keep searching until the frontier queue is empty

    while cell != start:
        shortest_path[bfs_path[cell]] = cell
        cell = bfs_path[cell]
    # return the search path, the shortest path from destination to start, and shortest path.
    return search_path, bfs_path, shortest_path


def dfs(my_maze):
    """
    Depth-First-Search algorithm to find the shortest path.
    :param
    my_maze: a randomly created, or loaded maze using pyamaze
    :return
    search_path: stores each iterated cell
    dfs_path: stores the shortest path from destination to start
    shortest_path: stores the shortest path obtained by traversing reversed_path
    """
    # initialize the starting cell as the bottom right corner, frontier queue,
    # path dictionary, and lists.
    start = (my_maze.rows, my_maze.cols)
    explored = [start]
    frontier = [start]
    dfs_path = {}
    search_path = []
    # keep searching until the frontier queue is empty
    while len(frontier) > 0:
        current_cell = frontier.pop()  # next cell
        search_path.append(current_cell)
        if current_cell == my_maze._goal:  # check whether the cell is the destination
            break
        for i in 'ESNW':
            if my_maze.maze_map[current_cell][i]:
                if i == 'E':
                    child_cell = (current_cell[0], current_cell[1] + 1)
                elif i == 'W':
                    child_cell = (current_cell[0], current_cell[1] - 1)
                elif i == 'S':
                    child_cell = (current_cell[0] + 1, current_cell[1])
                elif i == 'N':
                    child_cell = (current_cell[0] - 1, current_cell[1])
                if child_cell in explored:
                    continue

                explored.append(child_cell)
                frontier.append(child_cell)
                dfs_path[child_cell] = current_cell

    shortest_path = {}
    cell = my_maze._goal

    while cell != start:
        shortest_path[dfs_path[cell]] = cell
        cell = dfs_path[cell]
    return search_path, dfs_path, shortest_path


def dijkstra(my_maze):
    """
    Dijkstra algorithm to find the shortest path.
    :param
    my_maze: a randomly created, or loaded maze using pyamaze
    :return
    search_path: stores each iterated cell
    dijkstra_path: stores the shortest path from destination to start
    shortest_path: stores the shortest path obtained by traversing reversed_path
    """
    start = (my_maze.rows, my_maze.cols)
    frontier = PriorityQueue()
    frontier.put(start, False)
    previous_cell = {}
    current_costs = {}
    previous_cell[start] = None
    current_costs[start] = 0
    dijkstra_path = {}
    search_path = []
    while not frontier.empty():
        current_cell = frontier.get()
        search_path.append(current_cell)
        if current_cell == my_maze._goal:
            break
        for i in 'ESNW':
            if my_maze.maze_map[current_cell][i]:
                if i == 'E':
                    child_cell = (current_cell[0], current_cell[1] + 1)
                elif i == 'W':
                    child_cell = (current_cell[0], current_cell[1] - 1)
                elif i == 'S':
                    child_cell = (current_cell[0] + 1, current_cell[1])
                elif i == 'N':
                    child_cell = (current_cell[0] - 1, current_cell[1])
                new_cost = current_costs[current_cell] + 1
                if child_cell not in current_costs or new_cost < current_costs[child_cell]:
                    current_costs[child_cell] = new_cost
                    priority = new_cost
                    frontier.put(child_cell, priority)  # add the child cell to the frontier queue
                    previous_cell[child_cell] = current_cell
                    dijkstra_path[child_cell] = current_cell

    shortest_path = {}
    cell = my_maze._goal

    while cell != start:
        shortest_path[dijkstra_path[cell]] = cell
        cell = dijkstra_path[cell]
    return search_path, dijkstra_path, shortest_path


def random_walk(my_maze):
    """
    An algorithm to find the shortest path by roaming randomly.
    :param
    my_maze: a randomly created, or loaded maze using pyamaze
    :return
    search_path: stores each iterated cell
    random_path: stores the shortest path from destination to start
    shortest_path: stores the shortest path obtained by traversing reversed_path
    """
    start = (my_maze.rows, my_maze.cols)
    frontier = [start]
    explored = [start]
    random_path = {}
    search_path = []
    # choose a random cell from the frontier.
    while len(frontier) > 0:
        current_cell = random.choice(frontier)
        search_path.append(current_cell)
        frontier.remove(current_cell)
        if current_cell == my_maze._goal:
            break
        for i in 'ESNW':
            if my_maze.maze_map[current_cell][i]:
                if i == 'E':
                    child_cell = (current_cell[0], current_cell[1] + 1)
                elif i == 'W':
                    child_cell = (current_cell[0], current_cell[1] - 1)
                elif i == 'S':
                    child_cell = (current_cell[0] + 1, current_cell[1])
                elif i == 'N':
                    child_cell = (current_cell[0] - 1, current_cell[1])
                if child_cell in explored:
                    continue

                frontier.append(child_cell)
                explored.append(child_cell)
                random_path[child_cell] = current_cell

    shortest_path = {}
    cell = my_maze._goal
    while cell != start:
        shortest_path[random_path[cell]] = cell
        cell = random_path[cell]
    return search_path, random_path, shortest_path


def minimum_spanning_tree(my_maze):
    """
    Minimum spanning tree algorithm to find the shortest path.
    :param
    my_maze: a randomly created, or loaded maze using pyamaze
    :return
    search_path: stores each iterated cell
    mst_path: stores the shortest path from destination to start
    shortest_path: stores the shortest path obtained by traversing reversed_path
    """
    start = (my_maze.rows, my_maze.cols)
    frontier = [(0, start)]
    heapq.heapify(frontier)
    explored = [start]
    mst_path = {}
    search_path = []
    while len(frontier) > 0:
        # pop the lowest cost cell from the frontier.
        current_cost, current_cell = heapq.heappop(frontier)
        search_path.append(current_cell)
        if current_cell == my_maze._goal:
            break
        for i in 'ESNW':
            if my_maze.maze_map[current_cell][i]:
                if i == 'E':
                    child_cell = (current_cell[0], current_cell[1] + 1)
                elif i == 'W':
                    child_cell = (current_cell[0], current_cell[1] - 1)
                elif i == 'S':
                    child_cell = (current_cell[0] + 1, current_cell[1])
                elif i == 'N':
                    child_cell = (current_cell[0] - 1, current_cell[1])
                if child_cell in explored:
                    continue
                explored.append(child_cell)
                # push the child cell to the frontier with cost '1'.
                heapq.heappush(frontier, (1, child_cell))
                mst_path[child_cell] = current_cell

    shortest_path = {}
    cell = my_maze._goal

    while cell != start:
        shortest_path[mst_path[cell]] = cell
        cell = mst_path[cell]
    return search_path, mst_path, shortest_path


def find_paths(my_maze, algo):
    """
    A function to run the function that corresponds to chosen algorithm. It also calculates the
    the execution time of the corresponding algorithm, then displays it in text label format
    :param my_maze: a randomly created, or loaded maze using pyamaze
    :param algo: a function name chosen by user
    :return:
    search_path: stores each iterated cell
    reversed_path: stores the shortest path from destination to start
    shortest_path: stores the shortest path obtained by traversing reversed_path
    """
    # check which algorithm is passed, then call the corresponding function
    if algo == 'bfs':
        start_time = time.perf_counter()  # starting time of the algorithm function
        search_path, reversed_path, shortest_path = bfs(my_maze)
        end_time = time.perf_counter()  # ending time of the algorithm function
        execution_time = (end_time - start_time) * 1000000  # in ns
    elif algo == 'dfs':
        start_time = time.perf_counter()
        search_path, reversed_path, shortest_path = dfs(my_maze)
        end_time = time.perf_counter()
        execution_time = (end_time - start_time) * 1000000  # in ns
    elif algo == 'dijkstra':
        start_time = time.perf_counter()
        search_path, reversed_path, shortest_path = dijkstra(my_maze)
        end_time = time.perf_counter()
        execution_time = (end_time - start_time) * 1000000  # in ns
    elif algo == 'random':
        start_time = time.perf_counter()
        search_path, reversed_path, shortest_path = random_walk(my_maze)
        end_time = time.perf_counter()
        execution_time = (end_time - start_time) * 1000000  # in ns
    elif algo == 'mst':
        start_time = time.perf_counter()
        search_path, reversed_path, shortest_path = minimum_spanning_tree(my_maze)
        end_time = time.perf_counter()
        execution_time = (end_time - start_time) * 1000000  # in ns
    else:
        raise ValueError("Algorithm name is invalid!")

    # text labels to show corresponding algorithm's search path length, the shortest path length, execution time
    textLabel(test_maze, f'{algo} Search Path Length', len(search_path) + 1)
    textLabel(test_maze, f'{algo} Shortest Path Length', len(shortest_path) + 1)
    textLabel(test_maze, f'{algo} Execution Time', int(execution_time))

    return search_path, reversed_path, shortest_path


def visualize_paths(my_maze, paths, delay):
    """
    A function that is used to visualize each shortest path algorithm.
    :param my_maze: a randomly created, or loaded maze using pyamaze
    :param paths: a tuple that stores the search, reversed, and shortest paths
    :param delay: waiting time for agent to move in tracing part
    :return:
    """
    # adjust agents for, in order: search path, shortest path from dest. to start, shortest path
    search_agent = agent(test_maze, footprints=True, color='blue', filled=True)
    algo_agent = agent(test_maze, test_maze._goal[0], test_maze._goal[1], footprints=True, color='yellow',
                       goal=(test_maze.rows, test_maze.cols), filled="True")
    shortest_agent = agent(test_maze, footprints=True, color='red')

    # trace the path with the adjusted agent to provide a visualization
    my_maze.tracePath({search_agent: paths[0]}, delay=delay)
    my_maze.tracePath({algo_agent: paths[1]}, delay=delay)
    my_maze.tracePath({shortest_agent: paths[2]}, delay=delay)


if __name__ == '__main__':

    # choose whether creating maze from scratch or use the existing ones
    create_maze = input('Enter 1 if you want to create the maze yourself:\n')

    if create_maze == str(1):
        maze_size = int(input('Choose the size of maze ?x?:'))
        test_maze = maze(maze_size, maze_size)
        test_maze.CreateMaze(loopPercent=50)  # loopPercent is adjusted to add multiple paths
        # adjust the delay time to obtain optimal waiting time as the maze gets bigger
        if maze_size <= 10:
            delay = 400
        elif maze_size <= 20:
            delay = 100
        elif maze_size <= 30:
            delay = 25
        else:
            delay = 10

        paths = find_paths(test_maze, str(input("Choose an algorithm: bfs-dfs-dijkstra-random-mst:\n")))
        visualize_paths(test_maze, paths, delay)
        test_maze.run()
    else:
        algorithm = str(input("Choose an algorithm: bfs-dfs-dijkstra-random-mst:\n"))
        maze_sizes = [5, 10, 20, 30]
        agent_delay = [400, 100, 25, 10]
        size_index = int(input("Choose the size of maze --> 0:5x5, 1:10x10, 2:20x20, 3:30x30\n"))
        test_maze = maze(maze_sizes[size_index], maze_sizes[size_index])
        csv_file = 'maze_' + str(maze_sizes[size_index]) + 'x' + str(maze_sizes[size_index]) + '.csv'
        test_maze.CreateMaze(loadMaze=csv_file, loopPercent=50)

        paths = find_paths(test_maze, algorithm)

        visualize_paths(test_maze, paths, agent_delay[size_index])
        test_maze.run()
















