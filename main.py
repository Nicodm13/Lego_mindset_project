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

    # Draw grid
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
            start_x, start_y = 2, 3  # Assume start at (0,0)
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

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")
