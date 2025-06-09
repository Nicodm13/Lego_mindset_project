import os
import sys

# Add robot folder to sys.path before imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

import cv2
import socket
import threading
import contextlib
from robot.config import ROBOT_WIDTH, ROBOT_LENGTH, ROBOT_PORT
from robot.grid import Grid
from pathfinding.astar import AStar
from util.grid_overlay import GridOverlay
from util.find_balls import find_ping_pong_balls, draw_ball_detections

# --- Global Variables ---
robot_ip = "192.168.93.19"
client_socket = None
connection_failed = threading.Event()
connected = threading.Event()

start_node = None
visited_balls = set()
target_node = None
tsp_path = []
awaiting_response = False

# Ball detection state
detect_balls = False
ball_data = {
    'white_balls': {'pixels': [], 'grid': []},
    'orange_balls': {'pixels': [], 'grid': []}
}

# --- Grid & Webcam Setup ---
grid = Grid(1800, 1200, 17)

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
    global awaiting_response
    try:
        while connected.is_set():
            data = client_socket.recv(1024)
            if not data:
                print("Connection closed by robot.")
                connected.clear()
                break
            message = data.decode().strip()
            if message == "DONE":
                print("Received Command: DONE")
                awaiting_response = False
    except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
        print(f"Listener disconnected: {e}")
        connected.clear()

# --- Main Loop ---
printed_searching = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    original_frame = frame.copy()

    # Only detect balls after connected
    if connected.is_set():
        if not printed_searching:
            print("Searching for balls...")
            printed_searching = True

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

    cv2.imshow(WINDOW_NAME, frame)

    if connection_failed.is_set():
        print("Robot connection failed.")
        break

    # --- Automated Ball Path Execution ---
    if connected.is_set() and not awaiting_response:
        all_balls = ball_data['white_balls']['grid'] + ball_data['orange_balls']['grid']
        unvisited = [coord for coord in all_balls if coord not in visited_balls]

        if not tsp_path and unvisited:
            if not start_node:
                if grid_overlay.start_point:
                    sx, sy = grid_overlay.start_point
                    start_node = grid.get_node(sx, sy)
                else:
                    start_node = grid.get_node(0, 0)
                    print("Default start node used.")

            closest = AStar.get_closest_nodes(start_node, unvisited, grid, n=3)
            tsp_path.extend(AStar.tsp_brute_force(start_node, closest, grid))

        if tsp_path:
            next_node = tsp_path.pop(0)
            path = AStar.find_path(start_node, next_node, grid, robot_width=ROBOT_WIDTH, robot_length=ROBOT_LENGTH)
            if path:
                try:
                    path_str = " ".join(f"{{{node.x},{node.y}}}" for node in path)
                    move_command = f"MOVE {path_str}\n"
                    client_socket.sendall(move_command.encode())
                    print(f"Sent COMMAND: {move_command.strip()}")
                    visited_balls.add((next_node.x, next_node.y))
                    start_node = next_node
                    awaiting_response = True
                except Exception as e:
                    print(f"Error sending move: {e}")
            else:
                print("No valid path to next ball, skipping.")

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
        start_node = None
        target_node = None
        awaiting_response = False

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
