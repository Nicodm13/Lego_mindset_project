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

    def get_neighbours(self, node: Node, robot_width = 0, robot_length = 0):
        neighbours = []

        for _, (dx, dy) in Direction.ALL_DIRECTIONS:
            nx, ny = node.x + dx, node.y + dy
            neighbour = self.get_node(nx, ny)
            if neighbour and self.is_walkable(neighbour, robot_width, robot_length):
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

    

    def is_walkable(self, node: Node, robot_width: float = 0, robot_length: float = 0):
        """Check if a node is walkable, considering robot size."""
        if node.is_obstacle:
            return False

        # If robot size is specified, check area around node
        if robot_width > 0 and robot_length > 0:
            # Calculate how many nodes robot would cover
            nodes_per_width = int((robot_width / (self.width / self.density)) / 2)
            nodes_per_length = int((robot_length / (self.height / self.density)) / 2)

            for dx in range(-nodes_per_width, nodes_per_width + 1):
                for dy in range(-nodes_per_length, nodes_per_length + 1):
                    neighbor = self.get_node(node.x + dx, node.y + dy)
                    if neighbor and neighbor.is_obstacle:
                        return False

        return True

    def add_obstacle(self, node: Node):
        node.is_obstacle = True

    def remove_obstacle(self, node: Node):
        node.is_obstacle = False
    
    def get_dropoff(self, dropoffset: int, robot_width: int, robot_length: int):
        node_y = self.density // 2
        node_x = 0 if dropoffset == -1 else self.density - 1

        return self.get_node(node_x, node_y)

    def get_robot_size_in_nodes(self, robot_width: float, robot_length: float):
        """
        Returns the number of grid nodes the robot occupies in X (width) and Y (length) directions.
        """
        node_width_mm = self.width / self.density
        node_height_mm = self.height / self.density

        nodes_x = int((robot_width / node_width_mm) + 0.5)
        nodes_y = int((robot_length / node_height_mm) + 0.5)

        return nodes_x, nodes_y

