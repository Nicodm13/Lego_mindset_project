from typing import List
from pathfinding.node import Node
from pathfinding.direction import Direction
from pathfinding.grid import Grid

class Controller:
    grid: Grid
    
    def __init__(self, grid: Grid):
        self.grid = grid
    
    def set_target(target: Node):
        return
    
    def navigate_to_target(self, start: Node, target: Node) -> List[Node]:
        return
    
    def follow_path(self, path: List[Node]):
        i = 1
        while i < len(path):
            self.move_to(path[i-1], path[i])
            i += 1
    
    def move_to(self, start: Node, target: Node):
        xdiff = start.x - target.x
        ydiff = start.y - target.y
        
        direction = Direction.from_offset(xdiff, ydiff)
        self.rotate_to(direction)
        
        distance = self.grid.get_distance(start, target)
        self.drive(distance)
    
    def get_direction():
        return
    
    def drive(distance: float):
        return
    
    def rotate_to(offset: Direction):
        return