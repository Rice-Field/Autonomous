# Artificial Intelligence for Robotics, Udacity
# Jonathon Rice
# Python 3.6

from math import *
import random
import numpy as np

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
# grid = [[0, 0, 1, 0, 0, 0],
#         [0, 0, 1, 0, 0, 0],
#         [0, 0, 0, 0, 1, 0],
#         [0, 0, 1, 1, 1, 0],
#         [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    expand = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    x = init[0]
    y = init[1]
    g = 0
    # store delta action
    n = 0
    # calc vector distance from goal
    disx = goal[0] - init[0]
    disy = goal[1] - init[1]

    backtrack = []
    open = [[g, x, y, n]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand

    while not found and not resign:
        if len(open) == 0:
            resign = True
            return 'fail'
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            backtrack.append(next)
            x = next[1]
            y = next[2]
            g = next[0]
            if x == goal[0] and y == goal[1]:
                # expand[x][y] = '*'
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open.append([g2, x2, y2, i])
                            closed[x2][y2] = 1

    # start at solution action and map trail from original start                       
    for i in range((len(backtrack)-1),0,-1):
        policy = backtrack[i][3]
        curx = backtrack[i][1]
        cury = backtrack[i][2]
        dx = delta[policy][0]
        dy = delta[policy][1]

        # begin at solution, mark where it came from
        if i == (len(backtrack)-1):
            expand[curx-dx][cury-dy] = delta_name[policy]
        # mark previous location if action leads to a marked state that trails from solution
        if expand[curx][cury] != ' ':
            expand[curx-dx][cury-dy] = delta_name[policy]
    # put a goal symbol at end
    expand[goal[0]][goal[1]] = '*'
    return expand


# solution = search(grid, init, goal, cost)
# for i in range(len(solution)):
#     print(grid[i])
# print()
# for i in range(len(solution)):
#     print(solution[i])

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]


def Astar(grid,init,goal,cost,heuristic):
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    f = 0
    open = [[g, x, y, f]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0

    sg = 0
    sx = 0
    sy = 0
    sf = 999 # cost + heuristic
    si = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[1]
            y = next[2]
            g = next[0]
            f = next[3]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    # Check four move options, save 1 with lowest f score
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            f2 = g2 + heuristic[x2][y2]
                            if f2 < sf:
                                sf = f2
                                sg = g2
                                sx = x2
                                sy = y2
                                si = i
                # Did not encounter a usable policy
                if sf == 999:
                    return "Fail"
                # Pick best of 4 options
                open.append([sg, sx, sy, si])
                closed[sx][sy] = 1
                sf = 999

    return expand

expand = Astar(grid, init, goal, cost, heuristic)
print()
for i in range(len(expand)):
    print(expand[i])