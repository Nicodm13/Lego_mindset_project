import os
import sys
import platform
import logging

from robot import direction

# Add robot folder to sys.path before imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

import cv2
import socket
import threading
import numpy as np
from robot.config import ROBOT_WIDTH, ROBOT_LENGTH, ROBOT_PORT
from robot.grid import Grid
from pathfinding.astar import AStar
from util.grid_overlay import GridOverlay
from util.find_balls import find_ping_pong_balls, draw_ball_detections
from util.path_visualizer import draw_astar_path
#from util.find_robot import debug_robot_detection, find_robot, draw_robot_detection_overlay
from util.aruco_util import get_robot_position_and_angle
import time

# --- Global Variables ---
robot_ip = "172.20.10.9"
client_socket = None
connection_failed = threading.Event()
connected = threading.Event()

# Run HSV calibration at startup
'''print("Starting HSV calibration for robot detection...")
try:
    # Get HSV ranges from the debug interface
    hsv_ranges = debug_robot_detection()
    print("HSV calibration complete. Values will be used for robot detection.")
except Exception as e:
    print(f"HSV calibration failed: {e}")
    # Use default values if calibration fails
    hsv_ranges = (
        np.array([85, 20, 100]),  # lower_blue
        np.array([110, 150, 220]),  # upper_blue
        np.array([25, 40, 100]),   # lower_green
        np.array([40, 255, 255])   # upper_green
    )
    print("Using default HSV values for robot detection")
'''
start_node = None
visited_balls = set()
target_node = None
tsp_path = []
latest_path = []
awaiting_response = False
is_dropoff_time = False
ball_data = {
    'white_balls': {'pixels': [], 'grid': []},
    'orange_balls': {'pixels': [], 'grid': []}
}
robot_position = None
robot_orientation = None
orientation_corrected = False

# --- Grid & Webcam Setup ---
grid = Grid(1800, 1200, 17)

def handle_obstacle_marked(gx, gy):
    node = grid.get_node(gx, gy)
    if node:
        grid.add_obstacle(node)

grid_overlay = GridOverlay(grid.width, grid.height, grid.density, on_mark_obstacle=handle_obstacle_marked)

def open_webcam(index=2, width=1280, height=720):
    system = platform.system()
    logging.info(f"Detected OS: {system}")

    if system == "Windows":
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    elif system == "Darwin":
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    elif system == "Linux":
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    else:
        logging.error(f"Unsupported OS: {system}")
        return None

    if not cap.isOpened():
        logging.info("Initial webcam open failed. Retrying...")
        for i in range(5):
            time.sleep(0.5)
            cap.open(index)
            if cap.isOpened():
                break

    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        ret, _ = cap.read()
        if not ret:
            logging.error("Webcam opened but failed to read frames.")
            return None
        return cap
    else:
        logging.error("Could not open webcam.")
        return None

# --- OpenCV Window Setup ---
WINDOW_NAME = "Webcam Feed"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(WINDOW_NAME, grid_overlay.mouse_events)

# --- Robot Connection Thread ---
def connect_to_robot():
    global client_socket
    print(f"Connecting to robot at {robot_ip}:{ROBOT_PORT}...")
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((robot_ip, ROBOT_PORT))
        print("Connected to robot!")
        connected.set()

        # Initialize the robot
        init_command = f"INIT {grid.width} {grid.height} {grid.density}\n"
        client_socket.sendall(init_command.encode())
        print(f"Sent: {init_command.strip()}")

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

        threading.Thread(target=listen_for_robot, daemon=True).start()

    except Exception as e:
        print(f"Failed to connect: {e}")
        connection_failed.set()

# --- Robot Listener ---
def listen_for_robot():
    global awaiting_response, start_node
    try:
        while connected.is_set():
            data = client_socket.recv(1024)
            if not data:
                print("Connection closed by robot.")
                connected.clear()
                break

            message = data.decode().strip()

            if message.startswith("DONE"):
                gyro_angle = int(''.join(filter(str.isdigit, message)))
                print("Robot completed its task.")

                if robot_position:
                    x, y = robot_position
                    start_node = grid.get_node(x, y)
                    print(f"Updated robot position to: ({x}, {y})")

                awaiting_response = False

    except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
        print(f"Listener disconnected: {e}")
        connected.clear()
        
# --- Webcam Initialization ---
cap = open_webcam()
if cap is None:
    logging.error("Webcam could not be initialized. Exiting.")
    sys.exit(1)

# --- Main Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # original_frame = frame.copy()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    original_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) 

    # Detect balls and robot
    if connected.is_set():
        ball_data = find_ping_pong_balls(original_frame, grid_overlay)
        frame = draw_ball_detections(frame, ball_data)
    robot_orientation_old = robot_orientation
    # Detect robot
    robot_position, robot_orientation, frame = get_robot_position_and_angle(original_frame, grid_overlay)
    if robot_orientation == 0: # Only update the orientation if we get a valid value
        robot_orientation = robot_orientation_old
    # Draw grid and path overlays
    frame = grid_overlay.draw(frame)

    if latest_path:
        frame = draw_astar_path(frame, latest_path, grid_overlay)

    # Draw status + ball info text on top of all overlays
    text_y = 30
    line_height = 30
    status_text = "Press 'C' to Connect" if not connected.is_set() else "Connected"
    white_txt = f"White: {len(ball_data['white_balls']['grid'])} balls"
    orange_txt = f"Orange: {len(ball_data['orange_balls']['grid'])} balls"

    cv2.putText(frame, status_text, (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    text_y += line_height
    cv2.putText(frame, white_txt, (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    text_y += line_height
    cv2.putText(frame, orange_txt, (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)


    cv2.imshow(WINDOW_NAME, frame)

    if connection_failed.is_set():
        print("Robot connection failed.")
        break

    # --- Automated Ball Path Execution ---
    if connected.is_set() and not awaiting_response:
        if is_dropoff_time:
            dropoff_node = grid.get_dropoff(dropoffset=1, robot_width=ROBOT_WIDTH, robot_length=ROBOT_LENGTH)
            path = AStar.find_path(start_node, dropoff_node, grid, robot_width=ROBOT_WIDTH, robot_length=ROBOT_LENGTH)
            if path:
                latest_path = path
                try:
                    if robot_position and robot_orientation:
                        gx, gy = robot_position
                        orientation = round(robot_orientation, 2)
                        pose_msg = f"POSE {{{gx},{gy}}} {orientation}\n"
                        client_socket.sendall(pose_msg.encode())
                        print(f"Sent COMMAND: {pose_msg.strip()}")

                    path_str = " ".join(f"{{{node.x},{node.y}}}" for node in path)
                    move_command = f"DROPOFF {path_str}\n"
                    client_socket.sendall(move_command.encode())
                    print(f"Sent COMMAND: {move_command.strip()}")
                    awaiting_response = True
                    is_dropoff_time = False
                except Exception as e:
                    print(f"Error sending dropoff: {e}")
            else:
                print("No valid path to dropoff, skipping.")

        elif tsp_path:
            next_node = tsp_path.pop(0)
            path = AStar.find_path(start_node, next_node, grid, robot_width=ROBOT_WIDTH, robot_length=ROBOT_LENGTH)
            if path:
                latest_path = path
                try:
                    if robot_position and robot_orientation:
                        gx, gy = robot_position
                        orientation = round(robot_orientation, 2)
                        pose_msg = f"POSE {{{gx},{gy}}} {orientation}\n"
                        client_socket.sendall(pose_msg.encode())
                        print(f"Sent COMMAND: {pose_msg.strip()}")

                    path_str = " ".join(f"{{{node.x},{node.y}}}" for node in path)
                    move_command = f"MOVE {path_str}\n"
                    client_socket.sendall(move_command.encode())
                    print(f"Sent COMMAND: {move_command.strip()}")
                    visited_balls.add((next_node.x, next_node.y))
                    awaiting_response = True
                    is_dropoff_time = True
                except Exception as e:
                    print(f"Error sending move: {e}")
            else:
                print("No valid path to next ball, skipping.")


        elif not tsp_path:
            all_balls = ball_data['white_balls']['grid'] + ball_data['orange_balls']['grid']
            unvisited = [coord for coord in all_balls if coord not in visited_balls]

            if unvisited:
                if not start_node and robot_position:
                    sx, sy = robot_position
                    start_node = grid.get_node(sx, sy)

                closest = AStar.get_closest_nodes(start_node, unvisited, grid, n=1)
                tsp_path.extend(AStar.tsp_brute_force(start_node, closest, grid))
    


    # --- Key Handling ---
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c') and not connected.is_set():
        connection_failed.clear()
        threading.Thread(target=connect_to_robot, daemon=True).start()
    elif key == ord('r'):
        visited_balls.clear()
        tsp_path.clear()
        latest_path.clear()
        start_node = None
        target_node = None
        awaiting_response = False
        is_dropoff_time = False

        # Clear ball detections
        ball_data['white_balls']['pixels'].clear()
        ball_data['white_balls']['grid'].clear()
        ball_data['orange_balls']['pixels'].clear()
        ball_data['orange_balls']['grid'].clear()

        # Clear obstacles
        for col in grid.grid:
            for node in col:
                node.is_obstacle = False
        grid_overlay.obstacles.clear()

        # Reset start point
        grid_overlay.start_point = None

        # Send RESET to robot if connected
        if connected.is_set() and client_socket:
            try:
                client_socket.sendall(b"RESET\n")
                print("Sent COMMAND: RESET")
            except Exception as e:
                print(f"Failed to COMMAND: RESET - {e}")

        # Forcefully disconnect
        connected.clear()
        print("System reset. Waiting for 'C' to reconnect...")

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")