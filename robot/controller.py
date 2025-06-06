#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

from node import Node
from grid import Grid
from direction import Direction

import socket
import math

ROBOT_PORT = 9999  # Match PC client port
DEFAULT_HEADING = 0 # North

class Controller:
    def __init__(self):
        self.grid = None # Grid will be set during the initilization command

        # Physical specifications
        self.robot_width = 120 # mm
        self.robot_length = 150 # mm
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
    
    def navigate_to_target(self, path: list[Node]):
        """Follow a given path sent from PC."""
        if path and len(path) >= 2:
            self.follow_path(path)
        else:
            self.ev3.screen.print("Invalid or short path")

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
    
    def move_to_dropoff(self, dropoffset: int):
        """Move the robot to one of the dropoffs.

        Args:
            dropoff (int): The chosen dropoff to go to. `-1` for west dropoff, `1` for east dropoff.
                           _Also it's a portmanteau of dropoff and offset. Isn't it clever?!_
        """
        # density adjustment
        effective_density = self.grid.density
        if (effective_density % 2 == 0):
            print("WARNING: grid density is even -> cannot navigate to middle of grid")
            effective_density -= 1  # make density even such that it can be divided by 2
                                    # (subtraction as opposed to addition is arbitrary and likely unimportant)
        
        # identify node to navigate to
        node_y = effective_density / 2

        i = 0
        if dropoffset > 0:
            i += 1  # such that it starts from the east if east dropoff
        while not self.grid.grid.is_walkable(self.grid.grid[i * dropoffset][node_y]):
            i += 1
        
        node_x = i

        # move to dropoff
        self.navigate_to_target(self.grid.grid[node_x, node_y])
        
        # rotate mechanism towards dropoff
            # As this has not been designed yet, who knows which direction this is :)

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
            
        
    def rotate_to(self, target_angle: float, base_speed: int = 100):
        """Rotate robot to a specific heading angle in degrees.

        Args:
            target_angle (float): The target heading (0-360 degrees).
            base_speed (int, optional): Maximum speed during rotation. Defaults to 100.
        """
        # Calculate the minimal angle difference
        angle_diff = (target_angle - self.current_heading) % 360
        if angle_diff > 180:
            angle_diff -= 360  # Rotate shortest path

        if angle_diff == 0:
            return  # Already facing the right direction

        # Reset gyro sensor to 0
        self.gyro_sensor.reset_angle(0)

        # Determine rotation direction
        direction = 1 if angle_diff > 0 else -1

        while True:
            current_angle = self.gyro_sensor.angle()
            remaining = abs(angle_diff) - abs(current_angle)

            if remaining <= 0:
                break

            # Dynamically reduce speed as we get closer
            speed = max(30, int(base_speed * (remaining / abs(angle_diff))))

            self.left_motor.run(speed * direction)
            self.right_motor.run(-speed * direction)

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
                
                if command.startswith("INIT"):
                    parts = command.split()
                    if len(parts) == 4:
                        try:
                            width = int(parts[1])
                            height = int(parts[2])
                            density = int(parts[3])
                            self.grid = Grid(width, height, density)
                            self.ev3.screen.print("Grid Init: {},{},{}".format(width, height, density))
                            continue
                        except ValueError:
                            self.ev3.screen.print("Invalid INIT")

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
                        self.navigate_to_target(coords)
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
    controller = Controller()
    controller.start_server()

# === Run Main ===
if __name__ == '__main__':
    main()
