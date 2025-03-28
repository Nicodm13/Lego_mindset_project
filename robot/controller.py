#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, InfraredSensor, GyroSensor
from pybricks.parameters import Port, Stop

from node import Node
from grid import Grid
from direction import Direction


import socket
import math

ROBOT_PORT = 9999  # Match PC client port

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
        self.ir_sensor = InfraredSensor(Port.S1)
        self.gyro_sensor = GyroSensor(Port.S2)
        self.gyro_sensor.reset_angle(0)
        self.current_heading = 0  # Start facing "north" (or whatever default)

        # Display message
        self.ev3.screen.print("IR & Gyro Ready")
    
    def navigate_to_target(self, start: Node, target: Node):
        # For simplicity, assume direct move
        self.follow_path([start, target])
    
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
        
        distance = Grid.get_distance(start, target)
        self.drive(distance)

    def offset_to_angle(self, xdiff, ydiff):
        direction_name = Direction.from_offset(xdiff, ydiff)
        if direction_name:
            return Direction.ANGLE_MAP[direction_name]
        else:
            return self.current_heading  


    def drive(self, distance: float):
        """Drive the car forward

        Args:
            distance (float): distance to drive in millimeters
        """
        angle = self.distance_to_angle(distance)
        self.left_motor.run_target(100, angle, wait=False)
        self.right_motor.run_target(100, angle, wait=True)
        
    def distance_to_angle(self, distance: float):
        degrees_per_mm = 360 / (math.pi * self.wheel_diameter)
        return distance * degrees_per_mm
    
    def rotate_to(self, target_angle, speed=500):
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
        # Create socket
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
                
                if command.startswith("MOVE"):
                    parts = command.split()
                    if len(parts) != 5:
                        continue
                    
                    start_x, start_y = int(parts[1]), int(parts[2])
                    target_x, target_y = int(parts[3]), int(parts[4])
                    
                    start_node = self.grid.get_node(start_x, start_y)
                    target_node = self.grid.get_node(target_x, target_y)
                    
                    self.navigate_to_target(start_node, target_node)
        except Exception as e:
            self.ev3.screen.print("Error!")
            print("Error:", e)
        finally:
            conn.close()
            server_socket.close()
            self.ev3.screen.print("Server Closed")


# === MAIN FUNCTION ===

def main():
    grid = Grid(440, 440, 5)
    controller = Controller(grid)
    controller.start_server()

# === Run Main ===
if __name__ == '__main__':
    main()
