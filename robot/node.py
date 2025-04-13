class Node:
    def __init__(self, x, y):
        """
        Represents a single cell in the grid.

        Parameters:
        - x: The node's horizontal grid index
        - y: The node's vertical grid index
        """
        self.x = x
        self.y = y

        # Pathfinding costs
        self.g_cost = float('inf')  # Distance from start node
        self.h_cost = 0             # Heuristic distance to the end node
        self.f_cost = float('inf')  # Total cost (g_cost + h_cost)

        self.parent = None          # For path reconstruction
        self.walkable = True        # Will be set by Grid using shared_data

    def __lt__(self, other):
        """
        Enables comparison between nodes based on f_cost.
        Required for priority queues (heapq).
        """
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        """
        Optional: Allows 'node1 == node2' to compare by position.
        """
        return isinstance(other, Node) and self.x == other.x and self.y == other.y

    def __hash__(self):
        """
        Optional: Makes the Node hashable so it can be used in sets and dicts.
        """
        return hash((self.x, self.y))

    def __repr__(self):
        """
        Optional: Cleaner printing for debugging.
        """
        return f"Node({self.x}, {self.y}, walkable={self.walkable})"
