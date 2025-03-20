class Node:
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.g_cost = float('inf') # Distance from start
        self.h_cost = 0 # Heuristic estimate to target
        self.f_cost = float('inf') # Sum of g_cost and h_cost, used for pathfinding
        self.parent = None
