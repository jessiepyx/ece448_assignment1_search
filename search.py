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
    visited = set(cur)
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
    visited = set(cur)
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
    # TODO: Write your code here
    return []

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
