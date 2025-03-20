from enum import Enum

class Direction(Enum):
    NORTH = (0, -1)
    EAST = (1, 0)
    SOUTH = (0, 1)
    WEST = (-1, 0)
    
    NORTHEAST = (1, -1)
    SOUTHEAST = (1, 1)
    SOUTHWEST = (-1, 1)
    NORTHWEST = (-1, -1)

    @property
    def offset(self):
        return self.value
