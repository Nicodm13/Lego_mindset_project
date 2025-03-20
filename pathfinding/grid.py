import math
from .node import Node
from .direction import Direction

class Grid:
    def __init__(self, width, height, density):
        self.width = width          # Field width
        self.height = height        # Field height
        self.density = density      # How many nodes

        self.grid = []
        self.create_grid()

    def create_grid(self):
        self.num_nodes_x = self.density
        self.num_nodes_y = self.density

        self.grid = [[Node(x, y) for y in range(self.num_nodes_y)] for x in range(self.num_nodes_x)]

    def get_node(self, x: int, y: int) -> Node:
        if 0 <= x < self.num_nodes_x and 0 <= y < self.num_nodes_y:
            return self.grid[x][y]
        return None

    def get_neighbours(self, node: Node):
        neighbours = []
        for direction in Direction:
            dx, dy = direction.offset
            nx, ny = node.x + dx, node.y + dy
            neighbour = self.get_node(nx, ny)
            if neighbour:
                neighbours.append(neighbour)
        return neighbours
    
    def get_distance(node_a, node_b):
        dx = abs(node_a.x - node_b.x)
        dy = abs(node_a.y - node_b.y)
        if dx == 1 and dy == 1:
            # Diagonal
            return math.sqrt(2)
        else:
            # Straight
            return math.sqrt(dx ** 2 + dy ** 2)

    def is_walkable(node: Node):
        return
    
    def add_obstacle(node: Node):
        return