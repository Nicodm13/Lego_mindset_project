#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, InfraredSensor
from pybricks.parameters import Port, Button
from pybricks.media.ev3dev import SoundFile
import time

# Initialize EV3 Brick
ev3 = EV3Brick()

# Initialize motors
motor_A = Motor(Port.A)  # Controls left motor
motor_B = Motor(Port.B)  # Controls right motor
motor_C = Motor(Port.C)  # Controls spinner

# Initialize Infrared Sensor on Port.S1
ir_sensor = InfraredSensor(Port.S1)

# Display message
ev3.screen.print("Testing IR Sensor...")

# Function to play the tune
def play_tune():
    ev3.speaker.play_notes(
        ['E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'A4/8.', 'E4/8.',
         'E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'A4/8.', 'F4/8.',
         'R/8', 'F4/8', 'G4/8', 'F4/8', 'G4/8', 'G4/8.', 'C4/8.',
         'E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'G4/8.', 'B3/8.'],
        132
    )

# Infinite loop for debugging
while True:
    # Get button state from the remote (Channel 1)
    buttons = ir_sensor.buttons(1)

    # Clear screen
    ev3.screen.clear()

    # Run the spin motor
    motor_C.run(-500)

    # Steering control with limits
    if Button.LEFT_UP in buttons:
        motor_A.run(500)  # Drive forward
        ev3.screen.print("Turning Left")

    elif Button.LEFT_DOWN in buttons:
        motor_A.run(-500)  # Drive backward
        ev3.screen.print("Driving Backwards")
    else:
        motor_A.stop()

    # Movement control
    if Button.RIGHT_UP in buttons:
        motor_B.run(500)  # Drive forward
        ev3.screen.print("Driving Forward")

    elif Button.RIGHT_DOWN in buttons:
        motor_B.run(-500)  # Drive backward
        ev3.screen.print("Driving Backwards")

    else:
        motor_B.stop()

    # Short delay to avoid screen flicker
    time.sleep(0.2)
