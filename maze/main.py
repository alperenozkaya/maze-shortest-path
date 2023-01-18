from collections import deque
import heapq
from pyamaze import maze, COLOR, agent, textLabel
from queue import PriorityQueue
from collections import deque
import random


def bfs(my_maze):
    start = (my_maze.rows, my_maze.cols)
    frontier = deque()
    frontier.append(start)
    bfs_path = {}
    explored = [start]
    bSearch = []
    while len(frontier) > 0:
        current_cell = frontier.popleft()
        if current_cell == my_maze._goal:
            break
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
                if child_cell in explored:
                    continue
                frontier.append(child_cell)
                explored.append(child_cell)
                bfs_path[child_cell] = current_cell
                bSearch.append(child_cell)
    forward_path = {}
    cell = my_maze._goal
    while cell != start:
        forward_path[bfs_path[cell]] = cell
        cell = bfs_path[cell]
    return bSearch, bfs_path, forward_path


def dfs(my_maze):
    start = (my_maze.rows, my_maze.cols)
    explored = [start]
    frontier = [start]
    dfs_path = {}
    d_search = []
    while len(frontier) > 0:
        current_cell = frontier.pop()
        d_search.append(current_cell)
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
                frontier.append(child_cell)
                dfs_path[child_cell] = current_cell
    forward_path = {}
    cell = my_maze._goal
    while cell != start:
        forward_path[dfs_path[cell]] = cell
        cell = dfs_path[cell]
    return d_search, dfs_path, forward_path


def dijkstra(my_maze):
    start = (my_maze.rows, my_maze.cols)
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    dijkstra_path = {}
    dij_search = []
    while not frontier.empty():
        current_cell = frontier.get()
        dij_search.append(current_cell)
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
                new_cost = cost_so_far[current_cell] + 1
                if child_cell not in cost_so_far or new_cost < cost_so_far[child_cell]:
                    cost_so_far[child_cell] = new_cost
                    priority = new_cost
                    frontier.put(child_cell, priority)
                    came_from[child_cell] = current_cell
                    dijkstra_path[child_cell] = current_cell
    forward_path = {}
    cell = my_maze._goal
    while cell != start:
        forward_path[dijkstra_path[cell]] = cell
        cell = dijkstra_path[cell]
    return dij_search, dijkstra_path, forward_path


def random_walk(my_maze):
    start = (my_maze.rows, my_maze.cols)
    frontier = [start]
    explored = [start]
    random_path = {}
    random_search = []
    while len(frontier) > 0:
        current_cell = random.choice(frontier)
        random_search.append(current_cell)
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
    forward_path = {}
    cell = my_maze._goal
    while cell != start:
        forward_path[random_path[cell]] = cell
        cell = random_path[cell]
    return random_search, random_path, forward_path


def minimum_spanning_tree(my_maze):
    start = (my_maze.rows, my_maze.cols)
    frontier = [(0, start)]
    heapq.heapify(frontier)
    explored = [start]
    mst_path = {}
    mst_search = []
    while len(frontier) > 0:
        current_cost, current_cell = heapq.heappop(frontier)
        mst_search.append(current_cell)
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
                heapq.heappush(frontier, (1, child_cell))
                mst_path[child_cell] = current_cell
    forward_path = {}
    cell = my_maze._goal
    while cell != start:
        forward_path[mst_path[cell]] = cell
        cell = mst_path[cell]
    return mst_search, mst_path, forward_path


def find_paths(my_maze, algorithm):
    if algorithm == 'bfs':
        search_path, reversed_path, forward_path = bfs(my_maze)
    elif algorithm == 'dfs':
        search_path, reversed_path, forward_path = dfs(my_maze)
    elif algorithm == 'dijkstra':
        search_path, reversed_path, forward_path = dijkstra(my_maze)
    elif algorithm == 'random':
        search_path, reversed_path, forward_path = random_walk(my_maze)
    elif algorithm == 'mst':
        search_path, reversed_path, forward_path = minimum_spanning_tree(my_maze)
    else:
        raise ValueError("Invalid algorithm name")
    return search_path, reversed_path, forward_path


def visualize_paths(my_maze, paths):
    my_maze.tracePath({search_agent: paths[0]})
    my_maze.tracePath({algo_agent: paths[1]})
    my_maze.tracePath({shortest_agent: paths[2]})


if __name__ == '__main__':
    # Create a new maze with a size of 10x10
    test_maze = maze(30, 30)

    # Generate a random maze
    #test_maze.CreateMaze(loadMaze='maze--2023-01-18--14-47-20.csv')
    test_maze.CreateMaze(2, 2, loopPercent=10)

    paths = find_paths(test_maze, str(input("Choose an algorithm: bfs-dfs-dijkstra-random-mst")))
    # agents

    # adjust agents
    search_agent = agent(test_maze, footprints=True, color='blue', filled=True)
    algo_agent = agent(test_maze, test_maze._goal[0], test_maze._goal[1], footprints=True, color='yellow', goal=(test_maze.rows, test_maze.cols), filled="True")
    shortest_agent = agent(test_maze, footprints=True, color='red')




    shortest_path_text = textLabel(test_maze, 'A* path length', len(shortest_path) + 1)

    test_maze.run()








