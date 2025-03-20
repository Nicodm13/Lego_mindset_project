import os
import sys
import cv2
import socket
import threading

# Add robot folder to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

from robot.grid import Grid
from util.grid_util import GridUtil

# --- Global Variables ---
robot_ip = ""
start_node = None
target_nodes = []
client_socket = None
input_ready = threading.Event()
target_lock = threading.Lock()
connection_failed = threading.Event()

# --- Input Thread ---
def handle_inputs(grid):
    global robot_ip, start_node, target_nodes, client_socket

    # --- Get Robot IP ---
    robot_ip = input("Enter Robot IP address (e.g., 192.168.X.X): ").strip()
    if not robot_ip:
        print("No IP entered. Exiting.")
        connection_failed.set()
        return

    # --- Connect to Robot ---
    ROBOT_PORT = 9999
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to robot at {robot_ip}:{ROBOT_PORT}...")
    try:
        client_socket.connect((robot_ip, ROBOT_PORT))
        print("Connected to robot!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        connection_failed.set()
        return

    # --- Enter Start Node ---
    while True:
        try:
            start_x = int(input("Enter start node X: "))
            start_y = int(input("Enter start node Y: "))
            start_node = grid.get_node(start_x, start_y)
            print(f"Starting at ({start_x}, {start_y})")
            break
        except ValueError:
            print("Invalid input. Please enter integers.")

    input_ready.set()  # Signal ready!

    # --- Keep Asking Target Nodes ---
    while True:
        try:
            target_x = input("Enter target node X (or 'q' to quit): ").strip()
            if target_x.lower() == 'q':
                break
            target_x = int(target_x)

            target_y = int(input("Enter target node Y: ").strip())

            with target_lock:
                target_nodes.append(grid.get_node(target_x, target_y))

        except ValueError:
            print("Invalid input. Please enter integers.")
        except KeyboardInterrupt:
            break

    input_ready.clear()
    client_socket.close()
    print("Disconnected from robot.")

# --- Webcam Setup ---
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if not ret:
    print("Webcam couldn't be opened.")
    exit()

# --- Grid Setup ---
grid = Grid(440, 440, 5)
grid_util = GridUtil(grid)

# --- OpenCV Window ---
WINDOW_NAME = "Webcam Feed"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

# --- Start Input Thread Immediately ---
input_thread = threading.Thread(target=handle_inputs, args=(grid,))
input_thread.start()

# --- Main Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_height, frame_width = frame.shape[:2]
    frame = grid_util.draw(frame, window_width=frame_width, window_height=frame_height)
    cv2.imshow(WINDOW_NAME, frame)

    if connection_failed.is_set():
        print("Robot connection failed. Exiting.")
        break

    if input_ready.is_set():
        with target_lock:
            if target_nodes and start_node:
                target_node = target_nodes.pop(0)
                print(f"Driving from ({start_node.x}, {start_node.y}) to ({target_node.x}, {target_node.y})")
                command = f"MOVE {start_node.x} {start_node.y} {target_node.x} {target_node.y}\n"
                client_socket.sendall(command.encode())
                print(f"Sent command: {command.strip()}")
                start_node = target_node

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")
