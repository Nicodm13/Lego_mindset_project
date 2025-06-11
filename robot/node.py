class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g_cost = float('inf')  # Distance from start
        self.h_cost = 0             # Heuristic estimate to target
        self.f_cost = float('inf')  # Total cost = g + h
        self.parent = None
        self.is_obstacle = False

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return (self.x, self.y) == (other.x, other.y)

    def __gt__(self, other):
        return self.f_cost > other.f_cost

    def __hash__(self):
        return hash((self.x, self.y))

    def reset(self):
        self.parent = None
        self.g_cost = float('inf')
        self.h_cost = float('inf')
        self.f_cost = float('inf')