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
        self.spinner_motor.reset_angle(0)

        # Initialize Sensors
        self.gyro_sensor = GyroSensor(Port.S1)

        # Active socket connection (set in start_server)
        self.conn = None
     
        # Display message
        self.ev3.screen.print("Controller Ready")

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
        """
        Follow a path of nodes by driving to each of them in order.
        If `is_dropoff` is True, stop at the second-last node and perform drop-off.
        """
        spinner_started = False

        # Determine the final index to stop at
        stop_index = len(path) - 1 if not is_dropoff else len(path) - 2

        i = 1
        while i <= stop_index:
            if self.reset_requested:
                return

            nodes_remaining = stop_index - i + 1

            # Start spinner only if not dropping off and 2 nodes left
            if not spinner_started and nodes_remaining == 2 and not is_dropoff:
                print("Starting spinner")
                self.start_spinner(SPINNER_SPEED)
                spinner_started = True

            print("Step {}: From ({},{}) to ({},{}) - {} nodes remaining".format(
                i, path[i - 1].x, path[i - 1].y, path[i].x, path[i].y, nodes_remaining))

            self.move_to(path[i - 1], path[i])
            i += 1

        # Drop off the ball if requested
        if is_dropoff and len(path) >= 2:
            self.drop_off_ball()

        # Always reset spinner at the end
        self.reset_spinner()

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

        angle_difference = abs(self.angle_diff(angle, current_gyro))

        # Only rotate if angle difference is significant
        if angle_difference >= ROTATE_CORRECTION_THRESHOLD:
            self.rotate_to(angle)

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


    def rotate_to(self, target_angle: float):
        """
        Rotate to a precise angle using DriveBase and gyro correction.
        Ensures rotation lands exactly on 0°, 90°, 180°, or 270° with no drift.
        """
        try:
            self.drive_base.stop()

            # Normalize the target angle
            target_angle = self.normalize_angle(target_angle)
            current_angle = self.normalize_angle(self.gyro_sensor.angle())
            delta = self.angle_diff(target_angle, current_angle)

            print("Rotating from {}° to {}° (delta: {}°)".format(current_angle, target_angle, delta))

            # Use DriveBase for main rotation
            self.drive_base.settings(turn_rate=ROTATE_SPEED, turn_acceleration=ROTATE_ACCLERATION)
            self.drive_base.reset()
            self.drive_base.turn(delta)

            # Gyro correction loop
            max_attempts = 3
            attempts = 0

            while attempts < max_attempts:
                current_angle = self.normalize_angle(self.gyro_sensor.angle())
                delta = self.angle_diff(target_angle, current_angle)

                if abs(delta) <= ROTATE_CORRECTION_THRESHOLD:
                    break

                print("Correcting residual drift: {}°".format(delta))
                self.drive_base.turn(delta)
                attempts += 1

            # Reset gyro to clean up drift
            self.gyro_sensor.reset_angle(target_angle)

            wait(500)

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
    
    def reset_spinner(self):
        # Get the current spinner angle, modulo 360 to keep within one rotation
        current_angle = self.spinner_motor.angle() % 360

        # Calculate shortest path back to SPINNER_RESET_ANGLE (which is 0)
        target = 0 % 360
        diff = (target - current_angle + 180) % 360 - 180  # Result is in [-180, 180]

        print("Resetting spinner from {:.1f}° → rotating {:.1f}° to reach {}°".format(
            current_angle, diff, 0))

        # Perform relative turn
        self.spinner_motor.run_angle(SPINNER_SPEED, diff, then=Stop.BRAKE, wait=True)

        # Reset internal angle to keep it aligned with logic
        self.spinner_motor.reset_angle(0)

        print("Spinner reset complete.")

    def drop_off_ball(self):
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
            self.start_spinner(-SPINNER_SPEED)

            # Wait to let ball roll out
            wait(2000)  # 2 second

            # Reset spinner to pickup position
            self.reset_spinner()
            print("Ball dropped off and spinner reset.")

        except OSError as e:
            print("Drop-off error:", e)
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