class Direction:
    NORTH = (0, -1)
    EAST = (1, 0)
    SOUTH = (0, 1)
    WEST = (-1, 0)

    NORTHEAST = (1, -1)
    SOUTHEAST = (1, 1)
    SOUTHWEST = (-1, 1)
    NORTHWEST = (-1, -1)

    # List of all directions
    ALL_DIRECTIONS = [
        ('NORTH', NORTH),
        ('EAST', EAST),
        ('SOUTH', SOUTH),
        ('WEST', WEST),
        ('NORTHEAST', NORTHEAST),
        ('SOUTHEAST', SOUTHEAST),
        ('SOUTHWEST', SOUTHWEST),
        ('NORTHWEST', NORTHWEST),
    ]

    ANGLE_MAP = {
        'NORTH': 0,
        'NORTHEAST': 45,
        'EAST': 90,
        'SOUTHEAST': 135,
        'SOUTH': 180,
        'SOUTHWEST': 225,
        'WEST': 270,
        'NORTHWEST': 315,
    }

    @staticmethod
    def from_offset(xdiff, ydiff):
        for name, offset in Direction.ALL_DIRECTIONS:
            if offset == (xdiff, ydiff):
                return name
        return None
