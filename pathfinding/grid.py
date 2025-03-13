from node import Node

class Grid:
    def __init__(self, width, height):
        self.width = width
        self.heigth = height
        self.grid = [[Node(x,y) for y in range(height)] for x in range(width)]
        self.obstacles = set()

    def get_neighbours(node: Node):
        return
    
    def is_walkable(node: Node):
        return
    
    def add_obstacle(node: Node):
        return