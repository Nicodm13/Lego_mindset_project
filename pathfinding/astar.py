from robot.node import Node
from robot.grid import Grid

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
    def find_path(start: Node, target: Node, grid: Grid, robot_width=0, robot_length=0):
        def find_path(start: Node, target: Node, grid: Grid, robot_width=0, robot_length=0):
            """
            Find the optimal path from start node to target node using the A* algorithm.

            The algorithm finds the shortest path while considering:
            - Robot dimensions for collision avoidance
            - Special perpendicular approaches for targets on grid edges

            Parameters:
            ----------
            start : Node
                The starting node for path calculation
            target : Node
                The destination node to reach
            grid : Grid
                The grid environment containing all navigable and obstacle nodes
            robot_width : int, optional
                Width of the robot in grid units (default 0)
            robot_length : int, optional
                Length of the robot in grid units (default 0)

            Returns:
            -------
            list[Node]
                Ordered list of nodes from start to target representing the optimal path
                Empty list if no valid path exists
            """
        # Reset all nodes
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
                # If target is on an edge, enforce perpendicular approach
                edge_direction = None
                max_x, max_y = grid.num_nodes_x - 1, grid.num_nodes_y - 1

                if target.x == 0:
                    edge_direction = (1, 0)   # Approach from EAST
                elif target.x == max_x:
                    edge_direction = (-1, 0)  # Approach from WEST
                elif target.y == 0:
                    edge_direction = (0, 1)   # Approach from SOUTH
                elif target.y == max_y:
                    edge_direction = (0, -1)  # Approach from NORTH

                if edge_direction:
                    # Get how far back the robot must be
                    robot_size_x, robot_size_y = grid.get_robot_size_in_nodes(robot_width, robot_length)
                    approach_distance = robot_size_y // 2 if edge_direction in [(0, -1), (0, 1)] else robot_size_x // 2

                    # Determine the true approach node position
                    ax = target.x + edge_direction[0] * approach_distance
                    ay = target.y + edge_direction[1] * approach_distance
                    approach_node = grid.get_node(ax, ay)

                    # Validate all intermediate nodes
                    valid_approach = True
                    for i in range(approach_distance + 1):
                        bx = target.x + edge_direction[0] * i
                        by = target.y + edge_direction[1] * i
                        check_node = grid.get_node(bx, by)
                        if not check_node or not grid.is_walkable(check_node, robot_width, robot_length):
                            valid_approach = False
                            break

                    if valid_approach and approach_node and target != approach_node:
                        # Re-run path to safe approach point
                        approach_path = AStar.find_path(start, approach_node, grid, robot_width, robot_length)
                        if approach_path:
                            # Calculate direction from approach node to target (inverse of edge direction)
                            step_dx = -edge_direction[0]
                            step_dy = -edge_direction[1]

                            # Create the intermediate steps from approach node to target (exluding target node)
                            intermediate_nodes = []
                            for i in range(1, approach_distance + 1):
                                bx = approach_node.x + step_dx * i
                                by = approach_node.y + step_dy * i
                                step_node = grid.get_node(bx, by)
                                if not step_node:
                                    break
                                intermediate_nodes.append(step_node)

                            return approach_path + intermediate_nodes


                # Fallback: return direct path
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
    def find_path_fragment(length: int, start: Node, target: Node, grid: Grid, robot_width=0, robot_length=0):
        """
        Finds the first `length` nodes of the path from `start` to `target` using A* algorithm.
        """
        full_path = AStar.find_path(start, target, grid, robot_width, robot_length)
        return full_path[:length] if full_path else []


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

