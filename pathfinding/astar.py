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
    def get_valid_approach_directions(target: Node, grid: Grid, robot_width=0, robot_length=0):
        """Get all valid directions from which the robot can approach the target in a straight line"""
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # North, South, East, West
        valid_approaches = []
        
        robot_size_x, robot_size_y = grid.get_robot_size_in_nodes(robot_width, robot_length)
        # Use the larger dimension to ensure enough space for straight approach
        min_approach_distance = max(robot_size_x, robot_size_y, 2)  # At least 2 nodes for straight line
        
        for direction in directions:
            dx, dy = direction
            
            # Check if we can place approach nodes in this direction
            approach_nodes = []
            valid_direction = True
            
            for i in range(1, min_approach_distance + 1):
                ax = target.x + dx * i
                ay = target.y + dy * i
                approach_node = grid.get_node(ax, ay)
                
                if not approach_node or not grid.is_walkable(approach_node, robot_width, robot_length):
                    valid_direction = False
                    break
                    
                approach_nodes.append(approach_node)
            
            if valid_direction and len(approach_nodes) >= 2:
                valid_approaches.append((direction, approach_nodes))
        
        return valid_approaches

    @staticmethod
    def find_best_approach_path(start: Node, target: Node, grid: Grid, robot_width=0, robot_length=0):
        """Find the best approach path that ends with a straight line to the target"""
        valid_approaches = AStar.get_valid_approach_directions(target, grid, robot_width, robot_length)
        
        if not valid_approaches:
            return None
        
        best_path = None
        best_cost = float('inf')
        
        for direction, approach_nodes in valid_approaches:
            # Try to find path to the furthest approach node
            approach_target = approach_nodes[-1]  # Furthest node in this direction
            
            # Find path to approach point
            approach_path = AStar.find_path_internal(start, approach_target, grid, robot_width, robot_length)
            
            if approach_path:
                # Add the straight line approach nodes leading to the target
                # approach_nodes are ordered from target outward, so reverse to get path toward target
                straight_line_nodes = list(reversed(approach_nodes[:-1]))  # All but the furthest (which is approach_target)
                
                # Complete path: route to approach point + straight line approach + target
                total_path = approach_path + straight_line_nodes + [target]
                total_cost = len(total_path)
                
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_path = total_path
        
        return best_path

    @staticmethod
    def find_path_internal(start: Node, target: Node, grid: Grid, robot_width=0, robot_length=0):
        """Internal A* pathfinding without straight-line approach logic"""
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
    def find_path(start: Node, target: Node, grid: Grid, robot_width=0, robot_length=0):
        """Main pathfinding method that ensures straight-line approach to target"""
        
        # First try to find a path with straight-line approach
        path_with_approach = AStar.find_best_approach_path(start, target, grid, robot_width, robot_length)
        
        if path_with_approach:
            return path_with_approach
        
        # Fallback to original logic for edge cases or when straight approach isn't possible
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
                    approach_distance = robot_size_y if edge_direction in [(0, -1), (0, 1)] else robot_size_x

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
                        approach_path = AStar.find_path_internal(start, approach_node, grid, robot_width, robot_length)
                        if approach_path:
                            # Calculate direction from approach node to target (inverse of edge direction)
                            step_dx = -edge_direction[0]
                            step_dy = -edge_direction[1]

                            # Create the intermediate steps from approach node to target (excluding target node)
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