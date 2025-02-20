#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Initialize EV3 Brick
ev3 = EV3Brick()

# Initialize motor on a specific port (update if needed)
motor0 = Motor(Port.A)  # Change Port.A to the correct port if needed
motor1 = Motor(Port.B)  # Change Port.A to the correct port if needed

# Make a beep sound
ev3.speaker.beep()

# Run the motor at 500 degrees per second indefinitely
motor0.run(1000)
motor1.run(-1000)

# Display message
ev3.screen.print("Motor running...")

ev3.speaker.play_notes(
    ['E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'A4/8.', 'E4/8.',
     'E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'A4/8.', 'F4/8.',
     'R/8', 'F4/8', 'G4/8', 'F4/8', 'G4/8', 'G4/8.', 'C4/8.',
     'E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'G4/8.', 'B3/8.',
], 132)

# Stop the motor with a brake
motor0.stop()
motor1.stop()
ev3.screen.print("Motor stopped.")