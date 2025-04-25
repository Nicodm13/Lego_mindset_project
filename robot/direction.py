class Direction:
    NORTH = (0, -1)
    EAST = (1, 0)
    SOUTH = (0, 1)
    WEST = (-1, 0)

    # List of cardinal directions only
    ALL_DIRECTIONS = [
        ('NORTH', NORTH),
        ('EAST', EAST),
        ('SOUTH', SOUTH),
        ('WEST', WEST),
    ]

    ANGLE_MAP = {
        'NORTH': 0,
        'EAST': 90,
        'SOUTH': 180,
        'WEST': 270,
    }

    @staticmethod
    def from_offset(xdiff, ydiff):
        for name, offset in Direction.ALL_DIRECTIONS:
            if offset == (xdiff, ydiff):
                return name
        return None
