import math
import shared_data
from node import Node
from direction import Direction

class Grid:
    def __init__(self, width, height, density):
        """
        Initialize the grid with a specific width, height, and density (number of nodes per axis).
        """
        self.width = width              # Width of the field in units (e.g. pixels, meters)
        self.height = height            # Height of the field
        self.density = density          # How many divisions (nodes) along each axis

        self.grid = []                  # 2D list of Node objects
        self.create_grid()             # Initialize the grid

    def create_grid(self):
        """
        Constructs a 2D grid of nodes, marking each node as walkable if it's in shared_data.safe_coordinates.
        """
        self.num_nodes_x = self.density
        self.num_nodes_y = self.density
        self.grid = []

        for x in range(self.num_nodes_x):
            row = []
            for y in range(self.num_nodes_y):
                node = Node(x, y)
                # A node is walkable only if it's marked in shared_data by the OpenCV editor
                node.walkable = (x, y) in shared_data.safe_coordinates
                row.append(node)
            self.grid.append(row)

    def get_node(self, x: int, y: int) -> Node:
        """
        Returns the Node at grid coordinates (x, y), or None if out of bounds.
        """
        if 0 <= x < self.num_nodes_x and 0 <= y < self.num_nodes_y:
            return self.grid[x][y]
        return None

    def get_neighbours(self, node: Node):
        """
        Returns a list of neighboring nodes in 8 possible directions (N, NE, E, SE, S, SW, W, NW),
        using the Direction enum.
        """
        neighbours = []
        for direction in Direction:
            dx, dy = direction.offset
            nx, ny = node.x + dx, node.y + dy
            neighbour = self.get_node(nx, ny)
            if neighbour:
                neighbours.append(neighbour)
        return neighbours

    def get_distance(self, node_a: Node, node_b: Node):
        """
        Returns the physical distance between two nodes based on field dimensions.
        """
        dx = abs(node_a.x - node_b.x)
        dy = abs(node_a.y - node_b.y)

        if dx == dy == 0:
            return 0  # Same node
        if dy == 0:
            return self.width / self.density  # Horizontal move
        elif dx == 0:
            return self.height / self.density  # Vertical move
        else:
            # Diagonal move: use Pythagoras with scaled distances
            return math.sqrt((self.width / self.density) ** 2 + (self.height / self.density) ** 2)

    @staticmethod
    def is_walkable(node: Node):
        """
        Returns whether the given node is currently walkable.
        """
        return node.walkable

    @staticmethod
    def add_obstacle(node: Node):
        """
        Marks a given node as not walkable.
        """
        node.walkable = False
