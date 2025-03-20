import os
import cv2
import socket
from pathfinding.grid import Grid
from util.grid_util import GridUtil
from controller import Controller

# --- Constants ---
ROBOT_IP = '192.168.X.X'  # Replace with EV3 IP
ROBOT_PORT = 9999

# --- Webcam Setup ---
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
cap = cv2.VideoCapture(0)

ret, frame = cap.read()
if not ret:
    print("Webcam couldn't be opened.")
    exit()

# --- Grid Setup ---
grid = Grid(300, 300, 10)
grid_util = GridUtil(grid)
controller = Controller(grid)

# --- Terminal Input ---
start_x = int(input("Enter start node X: "))
start_y = int(input("Enter start node Y: "))
start_node = grid.get_node(start_x, start_y)
print(f"Starting at ({start_x}, {start_y})")

# --- Connect to Robot via Socket ---
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((ROBOT_IP, ROBOT_PORT))
print("Connected to robot.")

# --- OpenCV Window ---
WINDOW_NAME = "Webcam Feed"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(WINDOW_NAME, grid_util.handle_mouse)

# --- Main Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_height, frame_width = frame.shape[:2]
    frame = grid_util.draw(frame, window_width=frame_width, window_height=frame_height)
    cv2.imshow(WINDOW_NAME, frame)

    # Check for clicked node
    if grid_util.clicked_node:
        target_node = grid_util.clicked_node
        print(f"Navigating from ({start_node.x}, {start_node.y}) to ({target_node.x}, {target_node.y})")

        # Find path
        path = controller.navigate_to_target(start_node, target_node)

        # For each movement step, send command
        for i in range(1, len(path)):
            start = path[i-1]
            target = path[i]

            # Send MOVE command
            command = f"MOVE {start.x} {start.y} {target.x} {target.y}\n"
            client_socket.sendall(command.encode())
            print(f"Sent command: {command.strip()}")

        # Update start node
        start_node = target_node
        grid_util.clicked_node = None

    # Quit on 'q'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# --- Cleanup ---
client_socket.close()
cap.release()
cv2.destroyAllWindows()
