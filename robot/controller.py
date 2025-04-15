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
    
    def navigate_to_target(self, start: Node, target: Node):
        path = AStar.find_path(start, target, self.grid)
        if path and len(path) >= 2:
            self.follow_path(path)
        else:
            self.ev3.screen.print("No path found")

    def follow_path(self, path):
        i = 1
        while i < len(path):
            self.move_to(path[i-1], path[i])
            i += 1
    
    def move_to(self, start: Node, target: Node):
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
        """Drive the robot forward.

        Args:
            distance (float): Distance to drive in millimeters.
            speed (int, optional): Speed of wheel rotation in degrees/second. Defaults to 200.
        """
        self.left_motor.stop()
        self.right_motor.stop()
        
        distance_limit = 50  # mm

        start_angle = self.left_motor.angle()
        target_angle = self.distance_to_angle(distance)

        self.left_motor.run(speed)
        self.right_motor.run(speed)

        while True:
            # Check how far we've gone
            current_angle = self.left_motor.angle()
            if abs(current_angle - start_angle) >= abs(target_angle):
                break  # We've reached the target

            dist = self.us_sensor.distance()
            print("US: ", dist, "mm")

            if dist < distance_limit:
                self.on_wall_too_close()
                break

        self.left_motor.brake()
        self.right_motor.brake()

    def distance_to_angle(self, distance: float):
        degrees_per_mm = 360 / (math.pi * self.wheel_diameter)
        return distance * degrees_per_mm
    
    def on_wall_too_close(self):
        self.left_motor.brake()
        self.right_motor.brake()
        print("WARNING: Wall too close, stopping and continuing")
        
    
    def rotate_to(self, target_angle: float, speed: int = 100):
        """Rotate robot to a specific angle.

        Args:
            target_angle (float): Angle to rotate to in degrees, eg. 315 for perfect NW.
            speed (int, optional): Speed of wheel rotation in degrees/second. Defaults to 100.
        """
        angle_diff = (target_angle - self.current_heading) % 360
        if angle_diff > 180:
            angle_diff -= 360  # Shortest path

        # Reset gyro
        self.gyro_sensor.reset_angle(0)

        # Start motors
        if angle_diff > 0:
            self.left_motor.run(speed)
            self.right_motor.run(-speed)
        elif angle_diff < 0:
            self.left_motor.run(-speed)
            self.right_motor.run(speed)
        else:
            return  # Already aligned

        # Rotate, stop slightly early
        while abs(self.gyro_sensor.angle()) < abs(angle_diff) - 2:
            pass

        # Stop motors
        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)

        # Update heading
        self.current_heading = target_angle % 360

    def start_server(self):
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
