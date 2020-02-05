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
from itertools import count


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
    objs_tmp = sorted(objs, key=itemgetter(1))
    objs = sorted(objs_tmp, key=itemgetter(0))
    init_state = (pos, tuple(objs))
    cur_state = init_state
    dist = dict()
    dist[init_state] = 0
    heuristic = dict()
    heuristic[init_state] = min([abs(pos[0] - objs[i][0]) + abs(pos[1] - objs[i][1]) for i in range(4)]) + h + w + h
    frontier = []
    # tie breaking rule 1: states with fewer objectives have higher priority
    # tie breaking rule 2: newer states have higher priority
    counter = count()
    heappush(frontier, (heuristic[init_state], len(init_state[1]), -next(counter), init_state))
    explored = set()
    while len(frontier) > 0:
        f, _, _, cur_state = heappop(frontier)
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
        objs_tmp = sorted(objs, key=itemgetter(1))
        new_objs = sorted(objs_tmp, key=itemgetter(0))
        neighbors = maze.getNeighbors(pos[0], pos[1])
        for new_pos in neighbors:
            new_state = (new_pos, tuple(new_objs))
            # pushing expanded node to heap can lead to infinite loop
            if new_state not in explored:
                dist_tmp = dist[cur_state] + 1
                if new_state not in [x[3] for x in frontier]:
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
                    heappush(frontier, (dist[new_state] + heuristic[new_state], len(new_state[1]), -next(counter), new_state))
                    parent[new_state] = cur_state
                # can update node in frontier if distance is shorter
                elif dist_tmp < dist[new_state]:
                    dist[new_state] = dist_tmp
                    heappush(frontier, (dist[new_state] + heuristic[new_state], len(new_state[1]), -next(counter), new_state))
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
    parent = dict()
    path = []
    pos = maze.getStart()
    objs = maze.getObjectives()
    objs_tmp = sorted(objs, key=itemgetter(1))
    objs = sorted(objs_tmp, key=itemgetter(0))
    init_state = (pos, tuple(objs))
    cur_state = init_state
    dist = dict()
    dist[init_state] = 0

    # find minimum distance between two objectives (using bfs)
    def find_dist(x, y):
        parent = dict()
        cur = x
        frontier = deque()
        frontier.append(cur)
        # visited nodes include frontier
        visited = set()
        visited.add(cur)
        while len(frontier) > 0:
            cur = frontier.popleft()
            if cur == y:
                break
            neighbors = maze.getNeighbors(cur[0], cur[1])
            for i in neighbors:
                if i not in visited:
                    parent[i] = cur
                    frontier.append(i)
                    visited.add(i)
        dist = 0
        while cur != x:
            dist += 1
            cur = parent[cur]
        return dist

    # pre-compute pairwise distance of objectives
    # performance of real distance is much better than that of Manhattan distance
    obj_dist_map = dict()
    for i in range(len(objs) - 1):
        for j in range(i + 1, len(objs)):
            obj_dist_map[(objs[i], objs[j])] = find_dist(objs[i], objs[j])
            obj_dist_map[(objs[j], objs[i])] = obj_dist_map[(objs[i], objs[j])]

    # Prim's Algorithm for computing Minimum Spanning Tree
    # reference source: https://iq.opengenus.org/prim-minimum-spanning-tree-algorithm/
    def compute_mst(objs):
        cost = [1000000] * len(objs)
        cost[0] = 0
        parent = [0] * len(objs)
        visited = [False] * len(objs)
        for cnt in range(len(objs)):
            min_cost = 1000000
            min_index = 0
            for u in range(len(objs)):
                if cost[u] < min_cost and not visited[u]:
                    min_cost = cost[u]
                    min_index = u
            visited[min_index] = True
            for v in range(len(objs)):
                if not visited[v] and obj_dist_map[(objs[min_index], objs[v])] < cost[v]:
                    cost[v] = obj_dist_map[(objs[min_index], objs[v])]
                    parent[v] = min_index
        total_cost = 0
        for i in range(1, len(objs)):
            total_cost += obj_dist_map[(objs[i], objs[parent[i]])]
        return total_cost

    # cache MST to boost efficiency
    mst = dict()
    mst[tuple(objs)] = compute_mst(objs)
    heuristic = dict()
    heuristic[init_state] = min([abs(pos[0] - objs[i][0]) + abs(pos[1] - objs[i][1])
                                 for i in range(4)]) + mst[tuple(objs)]
    frontier = []
    # tie breaking rule 1: states with fewer objectives have higher priority
    # tie breaking rule 2: newer states have higher priority
    counter = count()
    heappush(frontier, (heuristic[init_state], -dist[init_state], len(init_state[1]), -next(counter), init_state))
    explored = set()
    while len(frontier) > 0:
        f, _, _, _, cur_state = heappop(frontier)
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
        objs_tmp = sorted(objs, key=itemgetter(1))
        new_objs = sorted(objs_tmp, key=itemgetter(0))
        neighbors = maze.getNeighbors(pos[0], pos[1])
        for new_pos in neighbors:
            new_state = (new_pos, tuple(new_objs))
            # pushing expanded node to heap can lead to infinite loop
            if new_state not in explored:
                dist_tmp = dist[cur_state] + 1
                if new_state not in [x[4] for x in frontier]:
                    dist[new_state] = dist_tmp
                    if tuple(new_objs) not in mst:
                        mst[tuple(new_objs)] = compute_mst(new_objs)
                    heuristic[new_state] = min([abs(new_pos[0] - new_objs[i][0]) + abs(new_pos[1] - new_objs[i][1])
                                                for i in range(len(objs))]) + mst[tuple(objs)]
                    heappush(frontier, (dist[new_state] + heuristic[new_state], -dist[new_state], len(new_state[1]),
                                        -next(counter), new_state))
                    parent[new_state] = cur_state
                # can update node in frontier if distance is shorter
                elif dist_tmp < dist[new_state]:
                    dist[new_state] = dist_tmp
                    heappush(frontier, (dist[new_state] + heuristic[new_state], -dist[new_state], len(new_state[1]),
                                        -next(counter), new_state))
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
    # Sub-optimal algorithm
    # Modified heuristic: MST -> 2 * MST
    parent = dict()
    path = []
    pos = maze.getStart()
    objs = maze.getObjectives()
    objs_tmp = sorted(objs, key=itemgetter(1))
    objs = sorted(objs_tmp, key=itemgetter(0))
    init_state = (pos, tuple(objs))
    cur_state = init_state
    dist = dict()
    dist[init_state] = 0

    # find minimum distance between two objectives (using bfs)
    def find_dist(x, y):
        parent = dict()
        cur = x
        frontier = deque()
        frontier.append(cur)
        # visited nodes include frontier
        visited = set()
        visited.add(cur)
        while len(frontier) > 0:
            cur = frontier.popleft()
            if cur == y:
                break
            neighbors = maze.getNeighbors(cur[0], cur[1])
            for i in neighbors:
                if i not in visited:
                    parent[i] = cur
                    frontier.append(i)
                    visited.add(i)
        dist = 0
        while cur != x:
            dist += 1
            cur = parent[cur]
        return dist

    # pre-compute pairwise distance of objectives
    # for this sub-optimal heuristic, Manhattan distance performs better
    obj_dist_map = dict()
    for i in range(len(objs) - 1):
        for j in range(i + 1, len(objs)):
            obj_dist_map[(objs[i], objs[j])] = abs(objs[i][0] - objs[j][0]) + abs(objs[i][1] - objs[j][1])
            obj_dist_map[(objs[j], objs[i])] = abs(objs[i][0] - objs[j][0]) + abs(objs[i][1] - objs[j][1])

    # Prim's Algorithm for computing Minimum Spanning Tree
    # reference source: https://iq.opengenus.org/prim-minimum-spanning-tree-algorithm/
    def compute_mst(objs):
        cost = [1000000] * len(objs)
        cost[0] = 0
        parent = [0] * len(objs)
        visited = [False] * len(objs)
        for cnt in range(len(objs)):
            min_cost = 1000000
            min_index = 0
            for u in range(len(objs)):
                if cost[u] < min_cost and not visited[u]:
                    min_cost = cost[u]
                    min_index = u
            visited[min_index] = True
            for v in range(len(objs)):
                if not visited[v] and obj_dist_map[(objs[min_index], objs[v])] < cost[v]:
                    cost[v] = obj_dist_map[(objs[min_index], objs[v])]
                    parent[v] = min_index
        total_cost = 0
        for i in range(1, len(objs)):
            total_cost += obj_dist_map[(objs[i], objs[parent[i]])]
        return total_cost

    # cache MST to boost efficiency
    mst = dict()
    mst[tuple(objs)] = compute_mst(objs)
    heuristic = dict()
    heuristic[init_state] = min([abs(pos[0] - objs[i][0]) + abs(pos[1] - objs[i][1])
                                 for i in range(4)]) + mst[tuple(objs)] * 2
    frontier = []
    # tie breaking rule 1: states with fewer objectives have higher priority
    # tie breaking rule 2: newer states have higher priority
    counter = count()
    heappush(frontier, (heuristic[init_state], len(init_state[1]), -next(counter), init_state))
    explored = set()
    while len(frontier) > 0:
        f, _, _, cur_state = heappop(frontier)
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
        objs_tmp = sorted(objs, key=itemgetter(1))
        new_objs = sorted(objs_tmp, key=itemgetter(0))
        neighbors = maze.getNeighbors(pos[0], pos[1])
        for new_pos in neighbors:
            new_state = (new_pos, tuple(new_objs))
            # pushing expanded node to heap can lead to infinite loop
            if new_state not in explored:
                dist_tmp = dist[cur_state] + 1
                if new_state not in [x[3] for x in frontier]:
                    dist[new_state] = dist_tmp
                    if tuple(new_objs) not in mst:
                        mst[tuple(new_objs)] = compute_mst(new_objs)
                    heuristic[new_state] = min([abs(new_pos[0] - new_objs[i][0]) + abs(new_pos[1] - new_objs[i][1])
                                                for i in range(len(objs))]) + mst[tuple(objs)] * 2
                    heappush(frontier, (dist[new_state] + heuristic[new_state], len(new_state[1]), -next(counter),
                                        new_state))
                    parent[new_state] = cur_state
                # can update node in frontier if distance is shorter
                elif dist_tmp < dist[new_state]:
                    dist[new_state] = dist_tmp
                    heappush(frontier, (dist[new_state] + heuristic[new_state], len(new_state[1]), -next(counter),
                                        new_state))
                    parent[new_state] = cur_state
    while cur_state != init_state:
        path.append(cur_state[0])
        cur_state = parent[cur_state]
    path.append(cur_state[0])
    path.reverse()
    return path
