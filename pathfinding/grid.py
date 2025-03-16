from .node import Node

class Grid:
    def __init__(self, width, height, density):
        self.width = width          # Field width
        self.height = height        # Field height
        self.density = density      # How many nodes

        self.grid = []
        self.recreate_grid()

    def recreate_grid(self):
        self.num_nodes_x = self.density
        self.num_nodes_y = self.density

        self.grid = [[Node(x, y) for y in range(self.num_nodes_y)] for x in range(self.num_nodes_x)]

    def get_neighbours(node: Node):
        return
    
    def is_walkable(node: Node):
        return
    
    def add_obstacle(node: Node):
        return