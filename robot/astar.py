from node import Node
from grid import Grid

import heapq

class AStar:
    @staticmethod
    def calculate_distance(node_a: Node, node_b: Node):
        dx = abs(node_a.x - node_b.x)
        dy = abs(node_a.y - node_b.y)
        return dx + dy

    @staticmethod
    def reverse_path(end_node: Node):
        path = []
        current = end_node
        while current:
            path.append(current)
            current = current.parent
        return path[::-1]

    @staticmethod
    def find_path(start: Node, target: Node, grid: Grid):
        open_set = []
        heapq.heappush(open_set, (0, start))
        start.g_cost = 0
        start.h_cost = AStar.calculate_distance(start, target)
        start.f_cost = start.h_cost

        closed_set = set()

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == target:
                return AStar.reverse_path(current)

            closed_set.add((current.x, current.y))

            for neighbor in grid.get_neighbours(current):
                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                tentative_g = current.g_cost + Grid.get_distance(grid, current, neighbor)

                if tentative_g < neighbor.g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = AStar.calculate_distance(neighbor, target)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    heapq.heappush(open_set, (neighbor.f_cost, neighbor))

        return []  
