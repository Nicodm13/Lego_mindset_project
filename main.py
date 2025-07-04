import os
import sys
import platform
import logging

from robot import direction

from util.aruco_util import get_robot_position_and_angle

# Add robot folder to sys.path before imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

import cv2
import socket
import threading
from robot.config import *
from robot.grid import Grid
from pathfinding.astar import AStar
from util.grid_overlay import GridOverlay
from util.find_balls import find_ping_pong_balls, draw_ball_detections
from util.path_visualizer import draw_astar_path
#from util.find_robot import debug_robot_detection, find_robot, draw_robot_detection_overlay
from util.aruco_util import get_robot_position_and_angle
import time

# --- Global Variables ---
robot_ip = "192.168.2.19"
client_socket = None
connection_failed = threading.Event()
connected = threading.Event()

start_node = None
visited_balls = set()
target_node = None
tsp_path = []
latest_path = []
awaiting_response = False
is_dropoff_time = False
robot_position = None
robot_orientation = None


# Ball detection state
detect_balls = False
ball_data = {
    'white_balls': {'pixels': [], 'grid': []},
    'orange_balls': {'pixels': [], 'grid': []}
}

# --- Grid & Webcam Setup ---
grid = Grid(GRID_WIDTH, GRID_HEIGHT, GRID_DENSITY)

def handle_obstacle_marked(gx, gy):
    node = grid.get_node(gx, gy)
    if node:
        grid.add_obstacle(node)

grid_overlay = GridOverlay(grid.width, grid.height, grid.density, on_mark_obstacle=handle_obstacle_marked)

def open_webcam(index=1, width=1280, height=720):
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
                parts = message.split()
                if len(parts) == 2:
                    try:
                        x_str, y_str = parts[1].split(",")
                        x, y = int(x_str), int(y_str)
                        # start_node = grid.get_node(x, y)
                        start_node = grid.get_node(*robot_position)
                        print(f"Updated robot position to: ({x}, {y})")
                    except Exception as e:
                        print(f"Failed to parse DONE position: {e}")
                else:
                    print("Received Command: DONE (no position)")

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
printed_searching = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)                    # Convert to grayscale
    original_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)           # For detection and overlays
    frame = original_frame.copy()                                     # Displayed frame will be grayscale + overlays

    robot_orientation_old = robot_orientation
    robot_position, robot_orientation, frame = get_robot_position_and_angle(original_frame, grid_overlay)
    if robot_orientation == 0: # Only update the orientation if we get a valid value
        robot_orientation = robot_orientation_old
    if connected.is_set():
            ball_data = find_ping_pong_balls(original_frame, grid_overlay)
            frame = draw_ball_detections(frame, ball_data)

            # Display ball coordinates
            white_txt = f"White: {len(ball_data['white_balls']['grid'])} balls"
            orange_txt = f"Orange: {len(ball_data['orange_balls']['grid'])} balls"
            cv2.putText(frame, white_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, orange_txt, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

    # Draw grid overlay after ball detection
    frame = grid_overlay.draw(frame)

    # Connection and detection status
    status_text = "Press 'C' to Connect" if not connected.is_set() else "Connected"
    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    # Draw any paths found by A*
    if latest_path:
        frame = draw_astar_path(frame, latest_path, grid_overlay)

    cv2.imshow(WINDOW_NAME, frame)

    if connection_failed.is_set():
        print("Robot connection failed.")
        break

    # --- Automated Ball Path Execution ---
    if connected.is_set() and not awaiting_response:
        if is_dropoff_time:
            dropoff_node = grid.get_dropoff(dropoffset=PREFERRED_DROPOFF, robot_width=ROBOT_WIDTH, robot_length=ROBOT_LENGTH)
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

                    is_fragment = len(path) > (FRAGMENT_SIZE * 2) + 1  # + 1 because start node is included in path
                    if is_fragment:
                        path = path[0:(FRAGMENT_SIZE*2) + 1]  # only include the first FRAGMENT_SIZE nodes
                        path_str = " ".join(f"{{{node.x},{node.y}}}" for node in path)
                        move_command = f"FRAGMENT {path_str}\n"
                    else:
                        path_str = " ".join(f"{{{node.x},{node.y}}}" for node in path)
                        move_command = f"DROPOFF {path_str}\n"
                    client_socket.sendall(move_command.encode())
                    print(f"Sent COMMAND: {move_command.strip()}")
                    awaiting_response = True
                    is_dropoff_time = is_fragment
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
                    is_fragment = len(path) > FRAGMENT_SIZE + 1  # + 1 because start node is included in path
                    if is_fragment:
                        path = path[0:FRAGMENT_SIZE + 1]  # only include the first FRAGMENT_SIZE nodes
                        path_str = " ".join(f"{{{node.x},{node.y}}}" for node in path)
                        move_command = f"FRAGMENT {path_str}\n"
                    else:
                        path_str = " ".join(f"{{{node.x},{node.y}}}" for node in path)
                        move_command = f"MOVE {path_str}\n"
                    client_socket.sendall(move_command.encode())
                    print(f"Sent COMMAND: {move_command.strip()}")
                    if not is_fragment:
                        visited_balls.add((next_node.x, next_node.y))
                        is_dropoff_time = True
                        print("it's dropoff time!!")
                    awaiting_response = True
                except Exception as e:
                    print(f"Error sending move: {e}")
            else:
                print("No valid path to next ball, skipping.")

        elif not tsp_path:
            all_balls = ball_data['white_balls']['grid'] + ball_data['orange_balls']['grid']
            unvisited = [coord for coord in all_balls if coord not in visited_balls]

            if unvisited:
                if not start_node:
                    start_node = grid.get_node(*robot_position) if robot_position else None
                closest = AStar.get_closest_nodes(start_node, unvisited, grid, n=1)
                tsp_path.extend(AStar.tsp_brute_force(start_node, closest, grid))


    # --- Key Handling ---
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c') and not connected.is_set():
        connection_failed.clear()
        threading.Thread(target=connect_to_robot, daemon=True).start()
    elif key == ord('d'):
        detect_balls = not detect_balls
        print("Ball detection enabled" if detect_balls else "Ball detection disabled")
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