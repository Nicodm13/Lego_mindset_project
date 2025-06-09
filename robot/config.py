# Physical robot dimensions in millimeters
ROBOT_WIDTH = 250
ROBOT_LENGTH = 320
WHEEL_DIAMETER = 56
AXLE_TRACK = 170 # Distance from the center of each wheel to the other

# Motion settings (degrees/second)
DRIVE_SPEED = 200
DRIVE_ACCELERATION = 800 # Acceleration should be SPEED * 4
ROTATE_SPEED = 100
ROTATE_ACCLERATION = 400 # Acceleration should be SPEED * 4
ROTATE_CORRECTION_THRESHOLD = 0.5
SPINNER_SPEED = 500

# Navigation
DEFAULT_HEADING = 0 # North
SAFE_DISTANCE_CHECK = 50

# Communication
ROBOT_PORT = 9999
