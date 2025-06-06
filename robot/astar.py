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
    def find_path(start: Node, target: Node, grid: Grid, robot_width = 0, robot_length = 0):
        # Reset nodes
        for col in grid.grid:
            for node in col:
                node.reset()

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

            for neighbor in grid.get_neighbours(current, robot_width, robot_length):
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

    @staticmethod
    def dijkstra(start: Node, grid: Grid):
        distances = {start: 0}
        queue = [(0, start)]

        for col in grid.grid:
            for node in col:
                node.reset()
        start.g_cost = 0

        while queue:
            cost, current = heapq.heappop(queue)
            for neighbor in grid.get_neighbours(current):
                if neighbor.is_obstacle:
                    continue
                new_cost = cost + grid.get_distance(current, neighbor)
                if new_cost < neighbor.g_cost:
                    neighbor.g_cost = new_cost
                    distances[neighbor] = new_cost
                    heapq.heappush(queue, (new_cost, neighbor))
        return distances

    @staticmethod
    def get_closest_nodes(start_node, ball_coords, grid, n=3):
        distances = AStar.dijkstra(start_node, grid)
        valid = []

        for gx, gy in ball_coords:
            node = grid.get_node(gx, gy)
            if node and node in distances:
                valid.append((distances[node], node))

        valid.sort()
        return [node for _, node in valid[:n]]

    @staticmethod
    def tsp_brute_force(start_node, targets, grid):
        best_order = None
        min_cost = float('inf')

        for perm in AStar.permutations(targets):
            total_cost = 0
            current = start_node
            for node in perm:
                path = AStar.find_path(current, node, grid)
                if not path:
                    total_cost = float('inf')
                    break
                total_cost += len(path)
                current = node
            if total_cost < min_cost:
                min_cost = total_cost
                best_order = perm

        return list(best_order)

    @staticmethod
    def permutations(iterable, r=None):
        pool = tuple(iterable)
        n = len(pool)
        r = n if r is None else r
        if r > n:
            return
        indices = list(range(n))
        cycles = list(range(n, n - r, -1))
        yield tuple(pool[i] for i in indices[:r])
        while n:
            for i in reversed(range(r)):
                cycles[i] -= 1
                if cycles[i] == 0:
                    indices[i:] = indices[i+1:] + indices[i:i+1]
                    cycles[i] = n - i
                else:
                    j = cycles[i]
                    indices[i], indices[-j] = indices[-j], indices[i]
                    yield tuple(pool[i] for i in indices[:r])
                    break
            else:
                return

