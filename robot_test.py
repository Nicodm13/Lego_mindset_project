#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, InfraredSensor, GyroSensor
from pybricks.parameters import Port, Button, Stop
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

# Initialize Gyro Sensor on Port.S2
gyro_sensor = GyroSensor(Port.S2)
gyro_sensor.reset_angle(0)

# Display message
ev3.screen.print("IR & Gyro Ready")

# ---------------------------
# Drive Method
def drive(speed, duration=None):
    motor_A.run(speed)
    motor_B.run(speed)
    if duration:
        time.sleep(duration)
        motor_A.stop(Stop.BRAKE)
        motor_B.stop(Stop.BRAKE)

# ---------------------------
# Rotate Method (positive = right turn, negative = left turn)
def rotate(target_angle, speed=150):
    ev3.screen.clear()
    ev3.screen.print("Rotating {}°".format(target_angle))

    # Reset gyro
    gyro_sensor.reset_angle(0)
    
    # Determine direction
    if target_angle > 0:
        # Right turn
        motor_A.run(speed)
        motor_B.run(-speed)
    else:
        # Left turn
        motor_A.run(-speed)
        motor_B.run(speed)

    # Keep turning until desired angle reached
    while abs(gyro_sensor.angle()) < abs(target_angle):
        pass  # Just wait
    
    # Stop motors
    motor_A.stop(Stop.BRAKE)
    motor_B.stop(Stop.BRAKE)
    
    ev3.screen.print("Done!")

# ---------------------------
# Function to play the tune
def play_tune():
    ev3.speaker.play_notes(
        ['E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'A4/8.', 'E4/8.',
         'E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'A4/8.', 'F4/8.',
         'R/8', 'F4/8', 'G4/8', 'F4/8', 'G4/8', 'G4/8.', 'C4/8.',
         'E4/8', 'E4/8', 'G4/8', 'E4/8', 'G4/8', 'G4/8.', 'B3/8.'],
        132
    )

# ---------------------------
# Main loop
while True:
    # Get button state from the remote (Channel 1)
    buttons = ir_sensor.buttons(1)

    # Default to empty list if buttons() returns None
    if buttons is None:
        buttons = []

    # Clear screen
    ev3.screen.clear()

    # Run the spinner motor
    motor_C.run(-500)

    # LEFT-UP Button → Rotate Left 90°
    if Button.LEFT_UP in buttons:
        rotate(-90)

    # LEFT-DOWN Button → Rotate Right 90°
    elif Button.LEFT_DOWN in buttons:
        rotate(90)

    # RIGHT-UP Button → Drive Forward for 1 sec
    if Button.RIGHT_UP in buttons:
        drive(500, duration=1)
        ev3.screen.print("Forward")

    # RIGHT-DOWN Button → Drive Backward for 1 sec
    elif Button.RIGHT_DOWN in buttons:
        drive(-500, duration=1)
        ev3.screen.print("Backward")

    # Stop motors if no buttons pressed (for safety)
    else:
        motor_A.stop()
        motor_B.stop()

    # Short delay to avoid flicker
    time.sleep(0.2)
