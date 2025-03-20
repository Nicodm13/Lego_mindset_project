#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, InfraredSensor, GyroSensor
from pybricks.parameters import Port, Button, Stop

from typing import List
from pathfinding.node import Node
from pathfinding.direction import Direction
from pathfinding.grid import Grid

class Controller:
    grid: Grid
    
    def __init__(self, grid: Grid):
        self.grid = grid
        
        # Initialize EV3 Brick
        self.ev3 = EV3Brick()

        # Initialize motors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        self.spinner_motor = Motor(Port.C)

        # Initialize Infrared Sensor on Port.S1
        self.ir_sensor = InfraredSensor(Port.S1)

        # Initialize Gyro Sensor on Port.S2
        self.gyro_sensor = GyroSensor(Port.S2)
        self.gyro_sensor.reset_angle(0)

        # Display message
        self.ev3.screen.print("IR & Gyro Ready")
    
    def set_target(target: Node):
        return
    
    def navigate_to_target(self, start: Node, target: Node) -> List[Node]:
        return
    
    def follow_path(self, path: List[Node]):
        i = 1
        while i < len(path):
            self.move_to(path[i-1], path[i])
            i += 1
    
    def move_to(self, start: Node, target: Node):
        xdiff = start.x - target.x
        ydiff = start.y - target.y
        
        angle = offset_to_angle(xdiff, ydiff)
        self.rotate_to(angle)
        
        distance = self.grid.get_distance(start, target)
        self.drive(distance)
        
        def offset_to_angle(xdiff, ydiff):
            angle = 180
            angle = angle + xdiff * 90
            angle = angle + ydiff * 45

            return angle
    
    def get_direction():
        return
    
    def drive(self, distance: float):
        angle = 360 * distance
        self.left_motor.run_angle(speed=60, rotation_angle=angle)
        self.right_motor.run_angle(speed=60, rotation_angle=angle)
    
    # Rotate Method (positive = right turn, negative = left turn)
    def rotate_to(self, target_angle, speed=150):
        self.ev3.screen.clear()
        self.ev3.screen.print("Rotating {}°".format(target_angle))

        # Reset gyro
        self.gyro_sensor.reset_angle(0)
        
        # Determine direction
        if target_angle > 0:
            # Right turn
            self.motor_left.run(speed)
            self.motor_right.run(-speed)
        else:
            # Left turn
            self.motor_left.run(-speed)
            self.motor_right.run(speed)

        # Keep turning until desired angle reached
        while abs(self.gyro_sensor.angle()) < abs(target_angle):
            pass  # Just wait
        
        # Stop motors
        self.motor_left.stop(Stop.BRAKE)
        self.motor_right.stop(Stop.BRAKE)
        
        self.ev3.screen.print("Done!")