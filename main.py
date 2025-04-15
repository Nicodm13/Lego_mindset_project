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
robot_ip = "192.168.71.19"
start_node = None
target_nodes = []
client_socket = None
input_ready = threading.Event()
target_lock = threading.Lock()
connection_failed = threading.Event()

# --- Input Thread ---
def handle_inputs(grid):
    global robot_ip, start_node, target_nodes, client_socket

    # --- Connect to Robot ---
    ROBOT_PORT = 9999
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Connecting to robot at {}:{}...".format(robot_ip, ROBOT_PORT))
    try:
        client_socket.connect((robot_ip, ROBOT_PORT))
        print("Connected to robot!")
    except Exception as e:
        print("Failed to connect: {}".format(e))
        connection_failed.set()
        return

    input_ready.set()  # Allow main loop to start

    print("Enter commands like:")
    print("  MOVE {1,2} {2,2}")
    print("  MOVE {1,2} {2,2} {3,2}")
    print("Type 'q' to quit.")

    while True:
        try:
            command = input("Command: ").strip()
            if command.lower() == 'q':
                break

            if command.startswith("MOVE"):
                coords = command.split()[1:]
                if len(coords) < 2:
                    print("MOVE requires at least 2 coordinates.")
                    continue

                parsed = []
                for c in coords:
                    c = c.strip("{}")
                    x, y = map(int, c.split(","))
                    node = grid.get_node(x, y)
                    if node is None:
                        print("Invalid node: {},{}".format(x, y))
                        break
                    parsed.append(node)

                if len(parsed) >= 2:
                    start_node = parsed[0]
                    target_nodes = parsed[1:]
                    client_socket.sendall((command + "\n").encode())
                    print("Sent command:", command)
                else:
                    print("Could not parse MOVE command.")
            else:
                print("Unknown command.")

        except Exception as e:
            print("Error:", e)
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
grid = Grid(1800, 1200, 4)
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

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")
