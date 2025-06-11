#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

from config import *

from node import Node
from grid import Grid
from direction import Direction

import socket

class Controller:
    def __init__(self):
        self.reset_requested = False # Flag to reset the robot
        self.grid = None # Grid will be set during the initilization command
        self.current_node = None  # The robot's current position

        # Physical specifications
        self.robot_width = ROBOT_WIDTH
        self.robot_length = ROBOT_LENGTH

        # Initialize EV3 Brick
        self.ev3 = EV3Brick()

        # Initialize motors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        self.drive_base = DriveBase(self.left_motor, self.right_motor, WHEEL_DIAMETER, AXLE_TRACK)
        self.spinner_motor = Motor(Port.C)

        # Initialize Sensors
        self.gyro_sensor = GyroSensor(Port.S2)
        self.us_sensor = UltrasonicSensor(Port.S1)

        # Active socket connection (set in start_server)
        self.conn = None

        # Display message
        self.ev3.screen.print("Controller Ready")

    def navigate_to_target(self, path: list[Node]):
        """Follow a given path sent from PC."""
        if(self.reset_requested):
            return

        if path and len(path) >= 2:
            print("Navigating to path with {} nodes".format(len(path)))
            self.current_node = path[0]
            self.follow_path(path)
        else:
            self.ev3.screen.print("Invalid or short path")

    def follow_path(self, path):
        """Follow a path of nodes by driving to each of them in order.

        Args:
            path (List[Node]): List of nodes to go to, in order, starting with the node the robot is currently on.
        """
        i = 1
        while i < len(path) - 1: # Stop at the previous node to the last one
            if(self.reset_requested):
                return
            print("Step {}: From ({},{}) to ({},{})".format(i, path[i-1].x, path[i-1].y, path[i].x, path[i].y))
            self.move_to(path[i-1], path[i])
            i += 1

        # Fetch the ball after reaching the last node
        if len(path) >= 2:
            self.fetch_ball(path[-1])

        # Notify PC when done
        if self.conn:
            try:
                print("Path complete, sending DONE")
                message = "DONE {},{}\n".format(self.current_node.x, self.current_node.y)
                self.conn.send(message.encode())
            except Exception as e:
                print("Failed to send DONE:", e)

    def move_to(self, start: Node, target: Node):
        """Rotate and drive to a **neighboring** node only if needed."""
        xdiff = target.x - start.x
        ydiff = target.y - start.y

        angle = self.offset_to_angle(xdiff, ydiff)
        current_gyro = self.gyro_sensor.angle()

        # Check if the target is already in the right direction
        if abs(angle - current_gyro) >= ROTATE_CORRECTION_THRESHOLD:
            self.rotate_to(angle)

        if self.reset_requested:
            self.left_motor.stop(Stop.BRAKE)
            self.right_motor.stop(Stop.BRAKE)
            return

        distance = self.grid.get_distance(start, target)
        self.drive(distance)
        self.current_node = target

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
        """Convert a rectangular offset to the corresponding angle, e.g., (1, -1) -> 45.

        Args:
            xdiff (int): Horizontal offset.
            ydiff (int): Vertical offset.

        Returns:
            int: Corresponding angle in degrees.

        Raises:
            ValueError: If the offset does not correspond to a valid direction.
        """
        try:
            direction_name = Direction.from_offset(xdiff, ydiff)
            return Direction.ANGLE_MAP[direction_name]
        except (KeyError, ValueError) as e:
            raise ValueError("Invalid offset ({}, {}) for direction lookup.".format(xdiff, ydiff)) from e
    
    def drive(self, distance: float):
        """Drive forward a specific distance using DriveBase.straight() with safe control."""
        try:
            self.drive_base.stop()

            self.drive_base.settings(DRIVE_SPEED, DRIVE_ACCELERATION)
   
            print("Driving: Distance={}, Speed={}".format(distance, DRIVE_SPEED))
            self.drive_base.straight(distance)

        except OSError as e:
            print("Drive EPERM error:", e)
            self.left_motor.stop(Stop.BRAKE)
            self.right_motor.stop(Stop.BRAKE)

        finally:
            self.drive_base.stop()

    def on_wall_too_close(self):
        """Behavior triggered when the ultrasonic sensor detects the robot being too close to a wall.
        What "too close" means is defined by the method calling this.
        """
        self.left_motor.brake()
        self.right_motor.brake()
        print("WARNING: Wall too close, stopping and continuing")


    def rotate_to(self, target_angle: float):
        """Rotate to target using gyro feedback with minimal unnecessary corrections."""
        try:
            self.drive_base.stop()
 
            current_gyro = self.gyro_sensor.angle()
            delta = self.angle_diff(target_angle, current_gyro)

            self.drive_base.settings(turn_rate=ROTATE_SPEED, turn_acceleration=ROTATE_ACCLERATION)
            self.drive_base.reset()

            # Primary rotation
            print("Rotating from {} to {} (delta: {})".format(current_gyro, target_angle, delta))
            self.drive_base.turn(delta)

        except OSError as e:
            print("Rotation EPERM error:", e)
            self.left_motor.stop(Stop.BRAKE)
            self.right_motor.stop(Stop.BRAKE)

        finally:
            self.drive_base.stop()
    
    def angle_diff(self, target, current):
        """Calculate minimal difference between two angles (degrees), result in [-180, 180]."""
        diff = (target - current + 180) % 360 - 180
        return diff

    def fetch_ball(self, target_node: Node):
        """Drives from the current node to the ball location and spins to pick it up.

        Args:
            target_node (Node): The location of the ball.
        """
        print("Fetching ball...")
        try:
            self.start_spinner(SPINNER_SPEED)
            self.drive_base.stop()
            self.drive_base.settings(PICKUP_SPEED, PICKUP_ACCELERATION)

            distance = self.grid.get_distance(self.current_node, target_node)
            print("Driving to ball at ({}, {}) with distance {}".format(target_node.x, target_node.y, distance))
            self.drive_base.straight(distance)

            self.current_node = target_node
        finally:
            self.reset_spinner()
            print("Ball fetched and spinner reset.")

    def start_spinner(self, speed: int = 500):
        """Start rotating the spinner.

        Args:
            speed (int, optional): Speed of spinner (in degrees/second). Defaults to 500.
        """
        self.spinner_motor.run(-speed)

    def reset_spinner(self):
        """Reset the spinner to the default 'up' position."""
        self.spinner_motor.stop(Stop.BRAKE)
        self.spinner_motor.run_target(-SPINNER_SPEED, SPINNER_RESET_ANGLE, Stop.BRAKE, wait=True)
        print("Spinner reset to default position.")

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
                                self.gyro_sensor.reset_angle(0)
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