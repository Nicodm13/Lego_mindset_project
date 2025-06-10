import os
import sys
import cv2
import socket
import threading
import contextlib
import sys
from util.find_balls import find_ping_pong_balls, draw_ball_detections
from util.find_robot import find_robot

# Add robot folder to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

from robot.grid import Grid
from util.grid_overlay import GridOverlay

# --- Global Variables ---
robot_ip = "192.168.59.19"
robot_port = 9999
client_socket = None
connection_failed = threading.Event()
connected = threading.Event()

# Ball detection state
detect_balls = False  # <-- Start as False
ball_data = {
    'white_balls': {'pixels': [], 'grid': []},
    'orange_balls': {'pixels': [], 'grid': []}
}

# Robot state tracking
robot_position = None
robot_orientation = None
orientation_corrected = False

# --- Grid & Webcam Setup ---
grid = Grid(1800, 1200, 12)

def handle_obstacle_marked(gx, gy):
    node = grid.get_node(gx, gy)
    if node:
        grid.add_obstacle(node)

grid_overlay = GridOverlay(grid.width, grid.height, grid.density, on_mark_obstacle=handle_obstacle_marked)

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if not ret:
    print("Webcam couldn't be opened.")
    exit()

# --- OpenCV Window Setup ---
WINDOW_NAME = "Webcam Feed"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(WINDOW_NAME, grid_overlay.mouse_events)

# --- Robot Connection Thread ---
def connect_to_robot():
    global client_socket
    print(f"Connecting to robot at {robot_ip}:{robot_port}...")
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((robot_ip, robot_port))
        print("Connected to robot!")
        connected.set()

        # Send obstacles
        obstacle_nodes = [
            node
            for col in grid.grid
            for node in col
            if node.is_obstacle
        ]
        if obstacle_nodes:
            obstacle_str = " ".join(f"{{{n.x},{n.y}}}" for n in obstacle_nodes)
            msg = f"OBSTACLE {obstacle_str}\n"
            client_socket.sendall(msg.encode())
            print(f"Sent: {msg.strip()}")
        else:
            print("No obstacles to send.")

    except Exception as e:
        print(f"Failed to connect: {e}")
        connection_failed.set()

def handle_robot_position():
    """Detect the robot's position and orientation and send correction if needed"""
    global robot_position, robot_orientation, orientation_corrected
    
    try:
        # Find robot position and orientation, passing the grid_overlay for grid-relative orientation
        robot_x, robot_y, robot_angle, robot_frame = find_robot(original_frame, grid_overlay)
        
        if robot_x is not None and robot_y is not None and robot_angle is not None:
            # Convert pixel coordinates to grid coordinates
            grid_x = int(robot_x / grid_overlay.cell_width)
            grid_y = int(robot_y / grid_overlay.cell_height)
            
            robot_position = (grid_x, grid_y)
            robot_orientation = robot_angle
            
            # Display robot information
            print(f"Robot detected at grid: ({grid_x}, {grid_y}), orientation: {robot_angle:.1f}°")
            
            # Only adjust orientation once after connection
            if connected.is_set() and not orientation_corrected:
                # Calculate angle adjustment (assuming robot should face North/0° in grid coordinates)
                target_orientation = 0  # North in grid coordinates
                adjustment = calculate_angle_adjustment(robot_angle, target_orientation)
                
                if abs(adjustment) > 5:  # Only adjust if more than 5 degrees off
                    adjust_command = f"ADJUST_ORIENTATION {adjustment}\n"
                    try:
                        client_socket.sendall(adjust_command.encode())
                        print(f"Sent orientation adjustment: {adjustment:.1f}°")
                        orientation_corrected = True
                    except Exception as e:
                        print("Failed to send orientation adjustment:", e)
                else:
                    print("Robot orientation is good enough, no adjustment needed")
                    orientation_corrected = True
            
            return robot_frame
    except Exception as e:
        print(f"Error detecting robot: {e}")
    
    return None

def calculate_angle_adjustment(current_angle, target_angle):
    """Calculate the shortest angle adjustment to reach the target orientation"""
    # Make sure angles are within 0-360
    current_angle = current_angle % 360
    target_angle = target_angle % 360
    
    # Calculate the difference
    adjustment = target_angle - current_angle
    
    # Take the shortest path
    if adjustment > 180:
        adjustment -= 360
    elif adjustment < -180:
        adjustment += 360
        
    return adjustment

# --- Tracking Move State ---
ball_targeted = False
printed_searching = False

# --- Suppress stdout temporarily ---
@contextlib.contextmanager
def suppress_stdout():
    with open(os.devnull, 'w') as fnull:
        old_stdout = sys.stdout
        sys.stdout = fnull
        try:
            yield
        finally:
            sys.stdout = old_stdout

# --- Main Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    original_frame = frame.copy()

    # Only detect balls and robot position after connected
    if connected.is_set():
        if not printed_searching:
            print("Searching for balls and robot position...")
            printed_searching = True

        # Detect robot position and orientation
        robot_display_frame = handle_robot_position()
        if robot_display_frame is not None:
            frame = robot_display_frame

        # Detect balls only after orientation is corrected
        if orientation_corrected:
            with suppress_stdout():
                ball_data = find_ping_pong_balls(original_frame, grid_overlay)

            frame = draw_ball_detections(frame, ball_data)

            # Display ball coordinates
            white_txt = f"White: {len(ball_data['white_balls']['grid'])} balls"
            orange_txt = f"Orange: {len(ball_data['orange_balls']['grid'])} balls"
            cv2.putText(frame, white_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, orange_txt, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            # Print coordinates in the grid to console
            if ball_data['white_balls']['grid']:
                print("White ball grid coordinates:", ball_data['white_balls']['grid'])
            if ball_data['orange_balls']['grid']:
                print("Orange ball grid coordinates:", ball_data['orange_balls']['grid'])

    # Draw grid overlay after ball detection
    frame = grid_overlay.draw(frame)

    # Connection and detection status
    status_text = "Press 'C' to Connect" if not connected.is_set() else "Connected"
    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    cv2.imshow(WINDOW_NAME, frame)

    if connection_failed.is_set():
        print("Robot connection failed.")
        break

    # --- After detection, send move command ---
    if connected.is_set() and orientation_corrected and not ball_targeted:
        target_coords = None

        if ball_data['white_balls']['grid']:
            target_coords = ball_data['white_balls']['grid'][0]
        elif ball_data['orange_balls']['grid']:
            target_coords = ball_data['orange_balls']['grid'][0]

        if target_coords and robot_position:
            start_x, start_y = robot_position
            target_x, target_y = target_coords

            move_command = f"MOVE {{{start_x},{start_y}}} {{{target_x},{target_y}}}\n"
            try:
                client_socket.sendall(move_command.encode())
                print(f"Ball found at ({target_x},{target_y})")
                print("Sent command:", move_command.strip())
                ball_targeted = True
            except Exception as e:
                print("Failed to send move command:", e)

    # --- Key Handling ---
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c') and not connected.is_set():
        connection_failed.clear()
        threading.Thread(target=connect_to_robot, daemon=True).start()
    elif key == ord('r'):
        # Reset orientation correction flag to force recalibration
        orientation_corrected = False
        print("Robot orientation detection reset")

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")
