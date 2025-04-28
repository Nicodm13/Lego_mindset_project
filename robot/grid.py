import math
from node import Node
from direction import Direction

class Grid:
    def __init__(self, width, height, density):
        self.width = width          # Field width (in mm)
        self.height = height        # Field height (in mm)
        self.density = density      # Number of nodes along each axis

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

        for _, (dx, dy) in Direction.ALL_DIRECTIONS:
            nx, ny = node.x + dx, node.y + dy
            neighbour = self.get_node(nx, ny)
            if neighbour and self.is_walkable(neighbour):
                neighbours.append(neighbour)

        return neighbours

    def get_distance(self, node_a: Node, node_b: Node):
        dx = abs(node_a.x - node_b.x)
        dy = abs(node_a.y - node_b.y)

        if dx + dy == 0:
            return 0
        elif dx + dy == 1:
            # Cardinal move
            return (self.width if dx else self.height) / self.density
        else:
            # Diagonal move, should never happen
            return float('inf')

    def is_walkable(self, node: Node):
        return not node.is_obstacle

    def add_obstacle(self, node: Node):
        node.is_obstacle = True
