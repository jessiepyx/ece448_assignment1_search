# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""

from collections import deque
from heapq import *


# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,extra)

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "extra": extra,
    }.get(searchMethod)(maze)


def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    parent = dict()
    path = []
    start = maze.getStart()
    cur = start
    frontier = deque()
    frontier.append(cur)
    visited = set()
    visited.add(cur)
    while len(frontier) > 0:
        cur = frontier.popleft()
        if maze.isObjective(cur[0], cur[1]):
            break
        neighbors = maze.getNeighbors(cur[0], cur[1])
        for i in neighbors:
            if i not in visited:
                parent[i] = cur
                frontier.append(i)
                visited.add(i)
    while cur != start:
        path.append(cur)
        cur = parent[cur]
    path.append(cur)
    path.reverse()
    return path


def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    parent = dict()
    path = []
    start = maze.getStart()
    cur = start
    target = maze.getObjectives()[0]
    g = dict()
    g[cur] = 0
    h = abs(cur[0] - target[0]) + abs(cur[1] - target[1])
    frontier = []
    heappush(frontier, (g[cur] + h, cur))
    visited = set()
    visited.add(cur)
    while len(frontier) > 0:
        f, cur = heappop(frontier)
        if maze.isObjective(cur[0], cur[1]):
            break
        neighbors = maze.getNeighbors(cur[0], cur[1])
        for i in neighbors:
            if i not in visited:
                parent[i] = cur
                g[i] = g[cur] + 1
                h = abs(i[0] - target[0]) + abs(i[1] - target[1])
                heappush(frontier, (g[i] + h, i))
                visited.add(i)
    while cur != start:
        path.append(cur)
        cur = parent[cur]
    path.append(cur)
    path.reverse()
    return path


def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.
        
    @param maze: The maze to execute the search on.
        
    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # state = (position, obj_list)
    # note: all dist are manhattan distance
    # suppose: w >= h
    # if len(obj_list) == 4: heuristic = dist_to_nearest_corner + h + w + h
    # if len(obj_list) == 3: heuristic = dist_to_nearest_corner_except_the_middle_one + h + w
    # if len(obj_list) == 2: heuristic = dist_to_nearest_corner + w
    # if len(obj_list) == 1: single point a*
    parent = dict()
    path = []
    h, w = maze.getDimensions()
    # Exclude walls
    h -= 3
    w -= 3
    if h > w:
        tmp = h
        h = w
        w = tmp
    pos = maze.getStart()
    objs = maze.getObjectives()
    init_state = (pos, tuple(objs))
    cur_state = init_state
    dist = dict()
    dist[init_state] = 0
    heuristic = min([abs(pos[0] - objs[i][0]) + abs(pos[1] - objs[i][1]) for i in range(4)]) + h + w + h
    frontier = []
    heappush(frontier, (heuristic, init_state))
    explored = set()
    explored.add(init_state)
    while len(frontier) > 0:
        f, cur_state = heappop(frontier)
        pos = cur_state[0]
        objs_tuple = cur_state[1]
        objs = list(objs_tuple)
        new_objs = objs
        if pos in new_objs:
            new_objs.remove(pos)
            if len(new_objs) == 0:
                break
        neighbors = maze.getNeighbors(pos[0], pos[1])
        for new_pos in neighbors:
            new_state = (new_pos, tuple(new_objs))
            if new_state not in explored or dist[cur_state] + 1 < dist[new_state]:
                parent[new_state] = cur_state
                dist[new_state] = dist[cur_state] + 1
                n = len(new_objs)
                if n == 4:
                    heuristic = dist[new_state] + min([abs(new_pos[0] - new_objs[i][0])
                                                       + abs(new_pos[1] - new_objs[i][1])
                                                       for i in range(4)]) + h + w + h
                elif n == 3:
                    # Exclude the middle corner
                    end_corners = []
                    unique_x = new_objs[0][0] ^ new_objs[1][0] ^ new_objs[2][0]
                    unique_y = new_objs[0][1] ^ new_objs[1][1] ^ new_objs[2][1]
                    for i in range(3):
                        if new_objs[i][0] == unique_x or new_objs[i][1] == unique_y:
                            end_corners.append(new_objs[i])
                    heuristic = dist[new_state] + min([abs(new_pos[0] - end_corners[i][0])
                                                       + abs(new_pos[1] - end_corners[i][1])
                                                       for i in range(2)]) + h + w
                elif n == 2:
                    heuristic = dist[new_state] + min([abs(new_pos[0] - new_objs[i][0]) + abs(new_pos[1] - new_objs[i][1])
                                                       for i in range(2)]) + w
                else:
                    heuristic = dist[new_state] + abs(new_pos[0] - new_objs[0][0]) + abs(new_pos[1] - new_objs[0][1])
                heappush(frontier, (heuristic, new_state))
                explored.add(new_state)
    while cur_state != init_state:
        path.append(cur_state[0])
        cur_state = parent[cur_state]
    path.append(cur_state[0])
    path.reverse()
    return path


def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []


def extra(maze):
    """
    Runs extra credit suggestion.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []
