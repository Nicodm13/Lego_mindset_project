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
        self.gyro_sensor = GyroSensor(Port.S1)
        self.gyro_sensor.reset_angle(0)

        # Active socket connection (set in start_server)
        self.conn = None

        # Display message
        self.ev3.screen.print("Controller Ready")

        # Reset the spinner
        self.reset_spinner()

    def navigate_to_target(self, path: list[Node], is_dropoff: bool):
        """Follow a given path sent from PC."""
        if(self.reset_requested):
            return

        if path and len(path) >= 2:
            print("Navigating to path with {} nodes".format(len(path)))
            self.current_node = path[0]
            self.follow_path(path, is_dropoff)
        else:
            self.ev3.screen.print("Invalid or short path")

    def follow_path(self, path, is_dropoff):
        """Follow a path of nodes by driving to each of them in order.

        Args:
            path (List[Node]): List of nodes to go to, in order, starting with the node the robot is currently on.
        """
        
        i = 1
        while i < len(path) - 1:
            if(self.reset_requested):
                return
            print("Step {}: From ({},{}) to ({},{})".format(i, path[i-1].x, path[i-1].y, path[i].x, path[i].y))
            self.move_to(path[i-1], path[i])
            i += 1

        # Fetch or drop off ball
        if not len(path) >= 2:
            pass
        elif is_dropoff:
            try:
                index = path.index(self.current_node)
                if index > 0:
                    previous_node = path[index - 1]
                else:
                    previous_node = self.current_node 
                self.drop_off_ball(self.current_node, previous_node)
            except ValueError:
                self.ev3.screen.print("Current node not in path")
        else:
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
        if abs(angle - current_gyro) > ROTATE_CORRECTION_THRESHOLD:
            self.rotate_to(angle)
            # TODO: Implement camera check

        if self.reset_requested:
            self.left_motor.stop(Stop.BRAKE)
            self.right_motor.stop(Stop.BRAKE)
            return

        distance = self.grid.get_distance(start, target)
        self.drive(distance)
        self.current_node = target

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

    def drive(self, distance: float, speed=DRIVE_SPEED, acceleration=DRIVE_ACCELERATION):
        """Drive forward a specific distance using DriveBase.straight() with safe control."""
        try:
            self.drive_base.stop()

            self.drive_base.settings(speed, acceleration)

            print("Driving: Distance={}, Speed={}".format(distance, speed))
            self.drive_base.straight(distance)

        except OSError as e:
            print("Drive EPERM error:", e)
            self.left_motor.stop(Stop.BRAKE)
            self.right_motor.stop(Stop.BRAKE)

        finally:
            self.drive_base.stop()

    def rotate_to(self, target_angle: float):
        """Rotate to target using gyro feedback with minimal unnecessary corrections."""
        try:
            self.drive_base.stop()

            # Normalize target to [0, 360)
            target_angle = self.normalize_angle(target_angle)

            current_gyro = self.normalize_angle(self.gyro_sensor.angle())
            delta = self.angle_diff(target_angle, current_gyro)

            print("Current: {:.1f}°, Target: {}°, Delta: {:.1f}°".format(
                current_gyro, target_angle, delta))

            if abs(delta) < ROTATE_CORRECTION_THRESHOLD:
                print("[Skipping rotation (within threshold of {}°)".format(
                    ROTATE_CORRECTION_THRESHOLD))
                return

            self.drive_base.settings(turn_rate=ROTATE_SPEED, turn_acceleration=ROTATE_ACCLERATION)
            self.drive_base.reset()

            self.drive_base.turn(delta)
            print("Rotating {}".format(delta))

            # Reset gyro to reflect new heading (avoid drift accumulation)
            self.gyro_sensor.reset_angle(target_angle)

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

    def normalize_angle(self, angle):
        """Normalize any angle to the [0, 360) range."""
        return angle % 360

    def fetch_ball(self, target_node: Node):
        """Rotates toward and drives to the ball from current node, and spins to pick it up."""
        print("Fetching ball...")

        try:
            self.start_spinner(SPINNER_SPEED)
            self.drive_base.stop()
            self.drive_base.settings(PICKUP_SPEED, PICKUP_ACCELERATION)

            # Rotate toward ball
            xdiff = target_node.x - self.current_node.x
            ydiff = target_node.y - self.current_node.y

            angle = self.offset_to_angle(xdiff, ydiff)
  
            self.rotate_to(angle)

            # Drive to ball
            distance = self.grid.get_distance(self.current_node, target_node)
            print("Driving to ball at ({}, {}) with distance {}".format(target_node.x, target_node.y, distance))
            self.drive_base.straight(distance)

            start_node = self.current_node
            self.current_node = target_node
            
            # Reverse a tile
            distance = self.grid.get_distance(self.current_node, start_node)

            self.drive(-distance, speed=DRIVE_SPEED, acceleration=DRIVE_ACCELERATION)
            self.current_node = start_node
            
        finally:
            self.reset_spinner()
            print("Ball fetched and spinner reset.")

    def drop_off_ball(self, start: Node, target: Node):
        """
        Drops off the ball by reversing the spinner briefly and then resetting.
        Then drives 1 node backward before sending DONE.
        """
        print("Dropping off ball...")

        try:
            if self.reset_requested:
                self.left_motor.stop(Stop.BRAKE)
                self.right_motor.stop(Stop.BRAKE)
                return

            # Spin backward to release ball
            self.spinner_motor.run_angle(SPINNER_SPEED, -90, then=Stop.BRAKE, wait=True)

            # Wait to let ball roll out
            wait(2000)  # 2 second

            # Reset spinner to pickup position
            self.reset_spinner()
            print("Ball dropped off and spinner reset.")

            # Get the distance to the previous node
            distance = self.grid.get_distance(start, target)

            # Reverse the distance
            self.drive(-distance, speed=DRIVE_SPEED, acceleration=DRIVE_ACCELERATION)
            self.current_node = target

        except OSError as e:
            print("Drop-off error:", e)
            self.spinner_motor.stop(Stop.BRAKE)


    def start_spinner(self, speed: int = 500):
        """Start rotating the spinner.

        Args:
            speed (int, optional): Speed of spinner (in degrees/second). Defaults to 500.
        """
        self.spinner_motor.run(-speed)

    def reset_spinner(self):
        """Reset the spinner to the default 'up' position using shortest rotation."""
        self.spinner_motor.stop(Stop.BRAKE)

        # Get the current spinner angle, modulo 360 to keep within one rotation
        current_angle = self.spinner_motor.angle() % 360

        # Calculate shortest path back to 0
        target = 0 
        diff = (target - current_angle + 180) % 360 - 180  # Result is in [-180, 180]

        print("Resetting spinner from {:.1f} rotating {:.1f} to reach {}".format(
            current_angle, diff, 0))

        # Perform relative turn
        self.spinner_motor.run_angle(SPINNER_SPEED, diff, then=Stop.BRAKE, wait=True)

        # Reset internal angle to keep it aligned with logic
        self.spinner_motor.reset_angle(0)

        print("Spinner reset complete.")


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

                    elif command.startswith("MOVE") or command.startswith("DROPOFF"):
                        is_dropoff = command.startswith("DROPOFF")
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
                            self.navigate_to_target(coords, is_dropoff)
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