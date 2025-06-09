#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

from config import (
    ROBOT_WIDTH, ROBOT_LENGTH, WHEEL_DIAMETER, DEFAULT_HEADING, SAFE_DISTANCE_CHECK, ROBOT_PORT,
    DRIVE_SPEED, ROTATE_BASE_SPEED, ROTATE_MIN_SPEED, SPINNER_SPEED
)

from node import Node
from grid import Grid
from direction import Direction

import socket
import math

class Controller:
    def __init__(self):
        self.reset_requested = False # Flag to reset the robot
        self.grid = None # Grid will be set during the initilization command


        # Physical specifications
        self.robot_width = ROBOT_WIDTH
        self.robot_length = ROBOT_LENGTH
        self.wheel_diameter = WHEEL_DIAMETER

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

        # Active socket connection (set in start_server)
        self.conn = None

        # Display message
        self.ev3.screen.print("Controller Ready")

    def navigate_to_target(self, path: list[Node]):
        """Follow a given path sent from PC."""
        if(self.reset_requested):
            return

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
            if(self.reset_requested):
                return
            self.move_to(path[i-1], path[i])
            i += 1

        # Notify PC when done
        if self.conn:
            try:
                self.conn.send(b"DONE\n")
            except Exception as e:
                print("Failed to send DONE:", e)

    def move_to(self, start: Node, target: Node):
        """Rotate and drive to a **neighboring** node.

        Args:
            start (Node): Node from which the robot starts.
            target (Node): Neighboring node to which the robot is to navigate and drive.
        """
        xdiff = target.x - start.x
        ydiff = target.y - start.y

        angle = self.offset_to_angle(xdiff, ydiff)
        self.rotate_to(angle, base_speed=ROTATE_BASE_SPEED)

        if self.reset_requested:
            self.left_motor.stop(Stop.BRAKE)
            self.right_motor.stop(Stop.BRAKE)
            return


        distance = self.grid.get_distance(start, target)
        self.drive(distance, speed=DRIVE_SPEED)

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
            effective_density -= 1

        # identify node to navigate to
        node_y = effective_density / 2

        i = 0
        if dropoffset > 0:
            i += 1
        while not self.grid.grid.is_walkable(self.grid.grid[i * dropoffset][node_y]):
            i += 1

        node_x = i

        # move to dropoff
        self.navigate_to_target(self.grid.grid[node_x, node_y])

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
        """Drive the robot forward a specific distance, checking continuously for obstacles.

        Args:
            distance (float): Distance to drive in millimeters.
            speed (int, optional): Speed of wheel rotation in degrees/second. Defaults to 200.
        """
        self.left_motor.stop()
        self.right_motor.stop()
        self.start_spinner(SPINNER_SPEED)

        angle = self.distance_to_angle(distance)

        # Start both motors non-blocking
        self.left_motor.run_angle(speed, angle, then=Stop.BRAKE, wait=False)
        self.right_motor.run_angle(speed, angle, then=Stop.BRAKE, wait=False)

        while self.left_motor.control.done() is False or self.right_motor.control.done() is False:
            if self.reset_requested:
                self.left_motor.stop(Stop.BRAKE)
                self.right_motor.stop(Stop.BRAKE)
                break
            if self.us_sensor.distance() < SAFE_DISTANCE_CHECK:
                self.on_wall_too_close()
                self.left_motor.stop(Stop.BRAKE)
                self.right_motor.stop(Stop.BRAKE)
                break
            wait(10)

        self.stop_spinner()

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
        """Rotate the robot to a specific heading using proportional control.

        Args:
            target_angle (float): The target heading (0-360 degrees).
            base_speed (int, optional): Maximum rotation speed. Defaults to 100.
        """
        current = self.current_heading
        delta = (target_angle - current + 360) % 360
        if delta > 180:
            delta -= 360  # Shortest path
        if abs(delta) < 0.1:
            return  # Already close enough

        self.gyro_sensor.reset_angle(0)
        wait(1000)  # Allow gyro to stabilize

        target_degrees = abs(delta)
        direction = 1 if delta > 0 else -1

        Kp = 2.5  # Proportional gain
        min_speed = ROTATE_MIN_SPEED

        while True:
            if self.reset_requested:
                self.left_motor.stop(Stop.BRAKE)
                self.right_motor.stop(Stop.BRAKE)
                return

            current_angle = abs(self.gyro_sensor.angle())
            remaining = target_degrees - current_angle
            if remaining <= 0.5:
                break

            speed = max(min_speed, min(base_speed, int(Kp * remaining)))
            self.left_motor.run(speed * direction)
            self.right_motor.run(-speed * direction)

        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)

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
        """Starts and runs the server on the robot to receive commands."""
        while True:
            server_socket = socket.socket()
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('', ROBOT_PORT))
            server_socket.listen(1)
            self.ev3.screen.clear()
            self.ev3.screen.print("Waiting for connection...")

            conn, addr = server_socket.accept()
            self.conn = conn
            self.ev3.screen.clear()
            self.ev3.screen.print("Connected!")

            try:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break  # client disconnected

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
                                self.reset_requested = False  # clear reset flag
                                self.ev3.screen.print("Grid Init: {},{},{}".format(width, height, density))
                            except ValueError:
                                self.ev3.screen.print("Invalid INIT")

                    elif command.startswith("OBSTACLE"):
                        parts = command.split()[1:]
                        for coord_str in parts:
                            coord_str = coord_str.strip("{}")
                            if "," in coord_str:
                                try:
                                    x_str, y_str = coord_str.split(",")
                                    x, y = int(x_str), int(y_str)
                                    node = self.grid.get_node(x, y)
                                    if node:
                                        self.grid.add_obstacle(node)
                                except Exception:
                                    self.ev3.screen.print("Invalid OBST")

                    elif command.startswith("MOVE"):
                        parts = command.split()
                        coords = []
                        for coord_str in parts[1:]:
                            coord_str = coord_str.strip("{}")
                            if "," in coord_str:
                                try:
                                    x_str, y_str = coord_str.split(",")
                                    x, y = int(x_str), int(y_str)
                                    node = self.grid.get_node(x, y)
                                    if node:
                                        coords.append(node)
                                except:
                                    self.ev3.screen.print("Invalid MOVE coord")

                        if len(coords) >= 2:
                            self.navigate_to_target(coords)
                        else:
                            self.ev3.screen.print("Invalid MOVE path")

                    elif command == "RESET":
                        self.reset_requested = True
                        self.left_motor.stop(Stop.BRAKE)
                        self.right_motor.stop(Stop.BRAKE)
                        self.spinner_motor.stop(Stop.BRAKE)
                        self.gyro_sensor.reset_angle(0)
                        self.current_heading = DEFAULT_HEADING
                        self.grid = None
                        self.ev3.screen.clear()
                        self.ev3.screen.print("Reset OK")
                        break  # exit this client session cleanly

            except Exception as e:
                self.ev3.screen.print("Error!")
                print("Error:", e)

            finally:
                conn.close()
                server_socket.close()
                wait(1000)  # Wait 1 second for port to be freed
                self.ev3.screen.clear()
                self.ev3.screen.print("Connection closed")
                wait(1000)  # brief pause before allowing reconnect


# === MAIN FUNCTION ===

def main():
    controller = Controller()
    controller.start_server()

# === Run Main ===
if __name__ == '__main__':
    main()