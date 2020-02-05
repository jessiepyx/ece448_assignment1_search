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
from operator import itemgetter


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
    # Single or multiple objectives
    # state = (position, obj_list)
    parent = dict()
    path = []
    pos = maze.getStart()
    objs = maze.getObjectives()
    init_state = (pos, tuple(objs))
    cur_state = init_state
    frontier = deque()
    frontier.append(init_state)
    # visited nodes include frontier
    visited = set()
    visited.add(cur_state)
    while len(frontier) > 0:
        cur_state = frontier.popleft()
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
            if new_state not in visited:
                parent[new_state] = cur_state
                frontier.append(new_state)
                visited.add(new_state)
    while cur_state != init_state:
        path.append(cur_state[0])
        cur_state = parent[cur_state]
    path.append(cur_state[0])
    path.reverse()
    return path


def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # Single objective
    parent = dict()
    path = []
    start = maze.getStart()
    cur = start
    target = maze.getObjectives()[0]
    g = dict()
    g[cur] = 0
    h = dict()
    h[cur] = abs(cur[0] - target[0]) + abs(cur[1] - target[1])
    frontier = []
    heappush(frontier, (g[cur] + h[cur], cur))
    explored = set()
    while len(frontier) > 0:
        f, cur = heappop(frontier)
        # if node is previously expanded, current cost must >= previous cost at this node
        if cur in explored:
            continue
        # only treat expanded nodes (not nodes in frontier) as visited, since nodes in frontier can be updated
        explored.add(cur)
        if maze.isObjective(cur[0], cur[1]):
            break
        neighbors = maze.getNeighbors(cur[0], cur[1])
        for i in neighbors:
            # pushing expanded node to heap can lead to infinite loop
            if i not in explored:
                g_tmp = g[cur] + 1
                if i not in [x[1] for x in frontier]:
                    g[i] = g_tmp
                    h[i] = abs(i[0] - target[0]) + abs(i[1] - target[1])
                    heappush(frontier, (g[i] + h[i], i))
                    parent[i] = cur
                # can update node in frontier if distance is shorter
                elif g_tmp < g[i]:
                    g[i] = g_tmp
                    heappush(frontier, (g[i] + h[i], i))
                    parent[i] = cur
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
    heuristic = dict()
    heuristic[init_state] = min([abs(pos[0] - objs[i][0]) + abs(pos[1] - objs[i][1]) for i in range(4)]) + h + w + h
    frontier = []
    heappush(frontier, (heuristic, init_state))
    explored = set()
    while len(frontier) > 0:
        f, cur_state = heappop(frontier)
        # if node is previously expanded, current cost must >= previous cost at this node
        if cur_state in explored:
            continue
        # only treat expanded nodes (not nodes in frontier) as visited, since nodes in frontier can be updated
        explored.add(cur_state)
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
            # pushing expanded node to heap can lead to infinite loop
            if new_state not in explored:
                dist_tmp = dist[cur_state] + 1
                if new_state not in [x[1] for x in frontier]:
                    dist[new_state] = dist_tmp
                    n = len(new_objs)
                    if n == 4:
                        heuristic[new_state] = min([abs(new_pos[0] - new_objs[i][0])
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
                        heuristic[new_state] = min([abs(new_pos[0] - end_corners[i][0])
                                                    + abs(new_pos[1] - end_corners[i][1])
                                                    for i in range(2)]) + h + w
                    elif n == 2:
                        heuristic[new_state] = min([abs(new_pos[0] - new_objs[i][0])
                                                    + abs(new_pos[1] - new_objs[i][1])
                                                    for i in range(2)]) + w
                    else:
                        heuristic[new_state] = abs(new_pos[0] - new_objs[0][0]) + abs(new_pos[1] - new_objs[0][1])
                    heappush(frontier, (dist[new_state] + heuristic[new_state], new_state))
                    parent[new_state] = cur_state
                # can update node in frontier if distance is shorter
                elif dist_tmp < dist[new_state]:
                    dist[new_state] = dist_tmp
                    heappush(frontier, (dist[new_state] + heuristic[new_state], new_state))
                    parent[new_state] = cur_state
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
    # state = (position, obj_list)
    # note: all dist are manhattan distance
    # suppose: w >= h
    # if len(obj_list) == 4: heuristic = dist_to_nearest_corner + h + w + h
    # if len(obj_list) == 3: heuristic = dist_to_nearest_corner_except_the_middle_one + h + w
    # if len(obj_list) == 2: heuristic = dist_to_nearest_corner + w
    # if len(obj_list) == 1: single point a*
    parent = dict()
    path = []
    pos = maze.getStart()
    objs = maze.getObjectives()
    init_state = (pos, tuple(objs))
    cur_state = init_state
    dist = dict()
    dist[init_state] = 0

    # TODO: A* distance instead of Manhattan distance
    # pre-compute pairwise distance of objectives
    obj_dist_map = dict()
    for x in objs:
        for y in objs:
            obj_dist_map[(x, y)] = abs(x[0] - y[0]) + abs(x[1] - y[1])

    # Prim's Algorithm for computing Minimum Spanning Tree
    # reference source: https://iq.opengenus.org/prim-minimum-spanning-tree-algorithm/
    def compute_mst(objs):
        cost = [1000000] * len(objs)
        cost[0] = 0
        parent = [0] * len(objs)
        visited = [False] * len(objs)
        for u in range(len(objs)):
            min_cost = 1000000
            min_index = 0
            for v in range(len(objs)):
                if cost[v] < min_cost and not visited[v]:
                    min_cost = cost[v]
                    min_index = v
            visited[min_index] = True
            for v in range(len(objs)):
                if obj_dist_map[(objs[u], objs[v])] < cost[v] and not visited[v]:
                    cost[v] = obj_dist_map[(objs[u], objs[v])]
                    parent[v] = u
        total_cost = 0
        for i in range(1, len(objs)):
            total_cost += obj_dist_map[(objs[i], objs[parent[i]])]
        return total_cost

    # cache MST to boost efficiency
    mst = dict()
    # sort the objective list to avoid repeated computation of the same MST
    objs_tmp = sorted(objs, key=itemgetter(1))
    objs_sorted = sorted(objs_tmp, key=itemgetter(0))
    mst[tuple(objs_sorted)] = compute_mst(objs_sorted)
    heuristic = dict()
    heuristic[init_state] = min([abs(pos[0] - objs[i][0]) + abs(pos[1] - objs[i][1])
                                 for i in range(4)]) + mst[tuple(objs_sorted)]
    frontier = []
    heappush(frontier, (heuristic, init_state))
    explored = set()
    while len(frontier) > 0:
        f, cur_state = heappop(frontier)
        # if node is previously expanded, current cost must >= previous cost at this node
        if cur_state in explored:
            continue
        # only treat expanded nodes (not nodes in frontier) as visited, since nodes in frontier can be updated
        explored.add(cur_state)
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
            # pushing expanded node to heap can lead to infinite loop
            if new_state not in explored:
                dist_tmp = dist[cur_state] + 1
                if new_state not in [x[1] for x in frontier]:
                    dist[new_state] = dist_tmp
                    new_objs_tmp = sorted(new_objs, key=itemgetter(1))
                    new_objs_sorted = sorted(new_objs_tmp, key=itemgetter(0))
                    if tuple(new_objs_sorted) not in mst:
                        mst[tuple(new_objs_sorted)] = compute_mst(new_objs_sorted)
                    heuristic[new_state] = min([abs(new_pos[0] - new_objs[i][0]) + abs(new_pos[1] - new_objs[i][1])
                                                for i in range(len(objs))]) + mst[tuple(new_objs_sorted)]
                    heappush(frontier, (dist[new_state] + heuristic[new_state], new_state))
                    parent[new_state] = cur_state
                # can update node in frontier if distance is shorter
                elif dist_tmp < dist[new_state]:
                    dist[new_state] = dist_tmp
                    heappush(frontier, (dist[new_state] + heuristic[new_state], new_state))
                    parent[new_state] = cur_state
    while cur_state != init_state:
        path.append(cur_state[0])
        cur_state = parent[cur_state]
    path.append(cur_state[0])
    path.reverse()
    return path


def extra(maze):
    """
    Runs extra credit suggestion.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # Greedy: find nearest objective using bfs one by one
    objs = maze.getObjectives()
    start = maze.getStart()
    cur = start
    path = [start]
    for j in range(len(objs)):
        frontier = deque()
        frontier.append(start)
        visited = set()
        visited.add(start)
        tmp_path = []
        parent = dict()
        while len(frontier) > 0:
            cur = frontier.popleft()
            if cur in objs:
                objs.remove(cur)
                break
            neighbors = maze.getNeighbors(cur[0], cur[1])
            for i in neighbors:
                if i not in visited:
                    parent[i] = cur
                    frontier.append(i)
                    visited.add(i)
        tmp_start = cur
        while cur != start:
            tmp_path.append(cur)
            cur = parent[cur]
        tmp_path.reverse()
        for p in tmp_path:
            path.append(p)
        start = tmp_start
    return path
