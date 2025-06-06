import os
import sys
import cv2
import socket
import threading
import contextlib
import sys
from util.find_balls import find_ping_pong_balls, draw_ball_detections

# Add robot folder to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

from robot.grid import Grid
from util.grid_overlay import GridOverlay
from robot.astar import AStar

# --- Global Variables ---
robot_ip = "192.168.93.19"
robot_port = 9999
start_node = None
visited_balls = set()
target_node = None
client_socket = None
connection_failed = threading.Event()
connected = threading.Event()

# Ball detection state
detect_balls = False  # <-- Start as False
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

    # Only detect balls after connected
    if connected.is_set():
        if not printed_searching:
            print("Searching for balls...")
            printed_searching = True

        with suppress_stdout():
            ball_data = find_ping_pong_balls(original_frame, grid_overlay)

        frame = draw_ball_detections(frame, ball_data)

        # Display ball coordinates
        white_txt = f"White: {len(ball_data['white_balls']['grid'])} balls"
        orange_txt = f"Orange: {len(ball_data['orange_balls']['grid'])} balls"
        cv2.putText(frame, white_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, orange_txt, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # Print coordinates in the grid to console
        #if ball_data['white_balls']['grid']:
            #print("White ball grid coordinates:", ball_data['white_balls']['grid'])
        #if ball_data['orange_balls']['grid']:
            #print("Orange ball grid coordinates:", ball_data['orange_balls']['grid'])

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
    if connected.is_set() and not ball_targeted:
        target_coords = None

        if ball_data['white_balls']['grid']:
            target_coords = ball_data['white_balls']['grid'][0]
        elif ball_data['orange_balls']['grid']:
            target_coords = ball_data['orange_balls']['grid'][0]

        if target_coords:
            # Use user-defined start point
            if grid_overlay.start_point:
                start_x, start_y = grid_overlay.start_point

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
    elif key == ord('d'):
        detect_balls = not detect_balls
        if detect_balls:
            print("Ball detection enabled")
        else:
            print("Ball detection disabled")
    elif key == ord('b'):
        if not start_node:
            print("No start node set. Using default (3,3) for testing.")
            start_node = grid.get_node(3, 3)

        all_balls = ball_data['white_balls']['grid'] + ball_data['orange_balls']['grid']
        unvisited = [coord for coord in all_balls if coord not in visited_balls]

        if not unvisited:
            print("No unvisited balls left.")
            continue

        closest_nodes = AStar.get_closest_nodes(start_node, unvisited, grid, n=3)
        best_order = AStar.tsp_brute_force(start_node, closest_nodes, grid)

        for node in best_order:
            command = f"MOVE {{{start_node.x},{start_node.y}}} {{{node.x},{node.y}}}"
            if connected.is_set() and client_socket:
                client_socket.sendall((command + "\n").encode())
                print("Sent to robot:", command)
            else:
                print("[Simulated] " + command)

            visited_balls.add((node.x, node.y))
            start_node = node
    elif key == ord('r'):
        visited_balls.clear()
        start_node = grid.get_node(3, 3)
        print("Visited balls cleared and Start node set to 0,0.")

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")
