#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

from node import Node
from grid import Grid
from direction import Direction
from astar import AStar

import socket
import math

ROBOT_PORT = 9999  # Match PC client port
DEFAULT_HEADING = 0 # North

class Controller:
    def __init__(self, grid: Grid):
        self.grid = grid
        
        # Physical specifications
        self.wheel_diameter = 55 # millimeters
        
        # Initialize EV3 Brick
        self.ev3 = EV3Brick()

        # Initialize motors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        self.spinner_motor = Motor(Port.C)

        # Initialize Sensors
        self.gyro_sensor = GyroSensor(Port.S2)
        self.gyro_sensor.reset_angle(0)
        self.us_sensor = UltrasonicSensor(Port.S1)
        self.current_heading = DEFAULT_HEADING

        # Display message
        self.ev3.screen.print("Controller Ready")
    
    def go_to_near_ball(self, start: Node, ball: Node):
        """Navigates to a tile near the ball, such that a future method can pick up the ball from a consistent distance from it.
        
        The method considers:
        - for the "**inner circle**", i.e. the robot-sized square around the ball (diameter same as robot):
            - **if a side has an obstacle:** all sides but opposite are blocked (the robot cannot access the ball from the same or perpendicular sides)
            - **if a corner has an obstacle:** the corner's two sides are blocked (obstacle is in front of the robot from those two sides, but behind the ball from the other two)
        - for the "**outer circle**", i.e. the area consisting of where the robot could be around the ball (diamater double the robot's + 1):
            - **if corner:** ignore (whether this has an obstacle is irrelevant for determining the sides from which the robot can access the ball)
            - **if side has obstacle:** this side is blocked (the robot cannot access the ball from this side because it would hit an obstacle)

        Args:
            start (Node): Node from which the robot starts.
            target (Node): Node at which the ball is, near to which the robot will navigate.
        """
        def sign(x: int):
            if x == 0:
                return 0
            return x / abs(x)
        
        blocked_sides = []
        size = 3 # size of robot in square diameter. Get this number from elsewhere when code is merged.
        halfsize = math.floor(size / 2)
        x = ball.x
        y = ball.y
        
        # DETERMINE BLOCKED SIDES
        for i in range(-size, size+1):
            for j in range(-size, size+1):
                # ignore ball
                if i == 0 and j == 0:
                    continue
                if self.grid.get_node(x+i, y+j).is_obstacle:
                    # INNER CIRCLE
                    if -halfsize <= i <= halfsize and -halfsize <= j <= halfsize:
                        if i != 0 and j != 0: # if corner
                            # add corner's sides
                            blocked_sides.append((sign(i), 0)) # REMEMBER TO CHANGE TO SIGN INSTEAD OF I
                            blocked_sides.append((0, sign(j)))
                        elif i == 0: # if horizontal wall
                            # add all sides but opposite side
                            blocked_sides.append((sign(i), 0))
                            blocked_sides.append((0, sign(j)))
                            blocked_sides.append((0, sign(-j)))
                        elif j == 0: # if vertical side
                            # add all sides but opposite side
                            blocked_sides.append((0, sign(j)))
                            blocked_sides.append((sign(i), 0))
                            blocked_sides.append((sign(-i), 0))
                    # OUTER CIRCLE
                    else:
                        # ignore corners
                        if (i < -halfsize or i > halfsize) and (j < -halfsize or j > halfsize):
                            continue
                        if i < -halfsize: blocked_sides.append(Direction.WEST)
                        elif i > halfsize: blocked_sides.append(Direction.EAST)
                        if j < -halfsize: blocked_sides.append(Direction.NORTH)
                        elif j > halfsize: blocked_sides.append(Direction.SOUTH)
        
        # DECIDE SIDE TO COME FROM
            # determine available directions
        all_directions = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]
        available_directions = [d for d in all_directions if d not in blocked_sides]
        
        if available_directions.count == 0:
            pass # give up
        
            # calculate offsets of start compared to ball
        offset_x = start.x - ball.x
        if (offset_x != 0):
            offset_x = offset_x / abs(offset_x)
            
        offset_y = start.y - ball.y
        if (offset_y != 0):
            offset_y = offset_y / abs(offset_y)
        
            # decide side to come from
        if (offset_x, 0) in available_directions: # come from a fitting side if possible
            from_direction = (offset_x, 0)
        elif (0, offset_y) in available_directions:
            from_direction = (0, offset_y)
        else: # else, come from any available side
            from_direction = available_directions[0]
        
        # START NAVIGATION
        new_x = ball.x + (from_direction[0] * (halfsize + 1))
        new_y = ball.y + (from_direction[1] * (halfsize + 1))
        new_target = self.grid.get_node(new_x, new_y)
        
        new_rotation = tuple(-x for x in from_direction)
        
            # start
        self.navigate_to_target(start, new_target)
        self.rotate_to(new_rotation)
    
    def navigate_to_target(self, start: Node, target: Node):
        """Have the robot drive from `start` to `target` following a path generated by :meth:`Astar.find_path`.

        Args:
            start (Node): Node from which the robot starts.
            target (Node): Node to which the robot is to navigate and drive.
        """        
        path = AStar.find_path(start, target, self.grid)
        if path and len(path) >= 2:
            self.follow_path(path)
        else:
            self.ev3.screen.print("No path found")

    def follow_path(self, path):
        """Follow a path of nodes by driving to each of them in order.

        Args:
            path (List[Node]): List of nodes to go to, in order, starting with the node the robot is currently on.
        """  
        i = 1
        while i < len(path):
            self.move_to(path[i-1], path[i])
            i += 1
    
    def move_to(self, start: Node, target: Node):
        """Rotate and drive to a **neighboring** node.

        Args:
            start (Node): Node from which the robot starts.
            target (Node): Neighboring node to which the robot is to navigate and drive.
        """        
        xdiff = target.x - start.x
        ydiff = target.y - start.y
        
        angle = self.offset_to_angle(xdiff, ydiff)
        self.rotate_to(angle)
        
        distance = self.grid.get_distance(start, target)
        self.drive(distance)

    def offset_to_angle(self, xdiff: int, ydiff: int) -> int:
        """Convert a rectangular offset to the corresponding angle, eg. `(1, -1)` -> `45`.

        Args:
            xdiff (int): Horizontal offset.
            ydiff (int): Vertical offset.

        Returns:
            int: Corresponding angle in degrees.
        """        
        direction_name = Direction.from_offset(xdiff, ydiff)
        if direction_name:
            return Direction.ANGLE_MAP[direction_name]
        else:
            return self.current_heading  


    def drive(self, distance: float, speed: int = 200):
        """Drive the robot forward. **NB: This implementation does not allow for reversing**.

        Args:
            distance (float): Distance to drive in millimeters.
            speed (int, optional): Speed of wheel rotation in degrees/second. Defaults to 200.
        """
        self.left_motor.stop()
        self.right_motor.stop()
        
        self.start_spinner()  # Start spinner when driving begins

        distance_limit = 50  # mm

        current_angle = self.left_motor.angle()
        goal_angle = self.distance_to_angle(distance)
        traveled_angle = 0

        self.left_motor.run(speed)
        self.right_motor.run(speed)

        while True:
            prev_angle = current_angle
            current_angle = self.left_motor.angle()
            
            if current_angle >= prev_angle:
                traveled_angle += current_angle - prev_angle
            else:
                traveled_angle += (360 - prev_angle) + current_angle
                
            if traveled_angle >= goal_angle:
                break

            dist = self.us_sensor.distance()
            if dist < distance_limit:
                self.on_wall_too_close()
                break

        self.left_motor.brake()
        self.right_motor.brake()
        self.stop_spinner()  # Stop spinner after driving ends

    def distance_to_angle(self, distance: float) -> float:
        """Convert distance to corresponding motor angle based on wheel circumference.

        Args:
            distance (float): distance in mm

        Returns:
            float: angle for motor to turn in degrees
        """        
        degrees_per_mm = 360 / (math.pi * self.wheel_diameter)
        return distance * degrees_per_mm
    
    def on_wall_too_close(self):
        """Behavior triggered when the ultrasonic sensor detects the robot being too close to a wall.
        What "too close" means is defined by the method calling this.
        """        
        self.left_motor.brake()
        self.right_motor.brake()
        print("WARNING: Wall too close, stopping and continuing")
            
        
    def rotate_to(self, target_angle: float, speed: int = 100):
        """Rotate the robot to a specified angle.

        Args:
            target_angle (float): Angle (in degrees) to rotate the robot to.
            speed (int, optional): Speed of the wheels doing the rotation (in degrees/second). Defaults to 100.
        """
        angle_diff = (target_angle - self.current_heading) % 360
        if angle_diff > 180:
            angle_diff -= 360  # Shortest path

        if angle_diff == 0:
            return  # Already aligned¨

        # Reset gyro
        self.gyro_sensor.reset_angle(0)

        # Start motors
        if angle_diff > 0:
            self.left_motor.run(speed)
            self.right_motor.run(-speed)
        elif angle_diff < 0:
            self.left_motor.run(-speed)
            self.right_motor.run(speed)

        # Rotate, stop slightly early
        while abs(self.gyro_sensor.angle()) < abs(angle_diff) - 2:
            pass

        # Stop motors
        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)

        # Update heading
        self.current_heading = target_angle % 360

    def start_spinner(self, speed: int = 500):
        """Start rotating the spinner.

        Args:
            speed (int, optional): Speed of spinner (in degrees/second). Defaults to 500.
        """        
        self.spinner_motor.run(-speed)

    def stop_spinner(self):
        """Stop rotating the spinner.
        """        
        self.spinner_motor.stop(Stop.BRAKE)

    def start_server(self):
        """Starts and runs the server on the robot to receive commands.
        """        
        server_socket = socket.socket()
        server_socket.bind(('', ROBOT_PORT))
        server_socket.listen(1)
        self.ev3.screen.print("Waiting for connection...")
        
        conn, addr = server_socket.accept()
        self.ev3.screen.clear()
        self.ev3.screen.print("Connected!")

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                command = data.decode().strip()
                self.ev3.screen.clear()
                self.ev3.screen.print("Cmd: {}".format(command))
                
                if command.startswith("OBSTACLE"):
                    parts = command.split()[1:]
                    for coord_str in parts:
                        coord_str = coord_str.strip("{}")
                        if "," in coord_str:
                            x_str, y_str = coord_str.split(",")
                            try:
                                x, y = int(x_str), int(y_str)
                                node = self.grid.get_node(x, y)
                                if node:
                                    self.grid.add_obstacle(node)
                            except:
                                self.ev3.screen.print("Invalid OBST")

                if command.startswith("MOVE"):
                    parts = command.split()
                    coords = []

                    for coord_str in parts[1:]:
                        coord_str = coord_str.strip("{}")
                        if "," in coord_str:
                            x_str, y_str = coord_str.split(",")
                            x, y = int(x_str), int(y_str)
                            node = self.grid.get_node(x, y)
                            if node:
                                coords.append(node)

                    if len(coords) >= 2:
                        start_node = coords[0]
                        target_node = coords[-1]
                        self.navigate_to_target(start_node, target_node)
                    else:
                        self.ev3.screen.print("Invalid MOVE path")
        except Exception as e:
            self.ev3.screen.print("Error!")
            print("Error:", e)
        finally:
            conn.close()
            server_socket.close()
            self.ev3.screen.print("Server Closed")

# === MAIN FUNCTION ===

def main():
    grid = Grid(1800, 1200, 4)
    controller = Controller(grid)
    controller.start_server()

# === Run Main ===
if __name__ == '__main__':
    main()
