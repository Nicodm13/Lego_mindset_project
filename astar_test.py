import os
import sys
import cv2
import threading

# Add robot folder to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

from robot.grid import Grid
from util.grid_util import GridUtil
from robot.astar import AStar
from robot.node import Node

# --- Grid Setup ---
grid = Grid(1800, 1200, 4)
grid_util = GridUtil(grid)

# --- Webcam Setup ---
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if not ret:
    print("Webcam couldn't be opened.")
    exit()

# --- Path result (shared with thread) ---
path = []
input_ready = threading.Event()

# --- Input Thread ---
def input_loop():
    global path
    print("Enter commands:")
    print("  MOVE {x1,y1} {x2,y2}  - to pathfind")
    print("  OBSTACLE {x,y}        - to place an obstacle")
    print("  q                     - to quit")

    while True:
        command = input("Command: ").strip()
        if command.lower() == 'q':
            input_ready.clear()
            break

        if command.startswith("MOVE"):
            coords = command.split()[1:]
            if len(coords) != 2:
                print("MOVE requires exactly 2 coordinates.")
                continue

            parsed = []
            for c in coords:
                c = c.strip("{}")
                try:
                    x, y = map(int, c.split(","))
                    node = grid.get_node(x, y)
                    if not node:
                        raise ValueError()
                    parsed.append(node)
                except:
                    print("Invalid coordinate:", c)
                    break

            if len(parsed) == 2:
                start_node, end_node = parsed
                path = AStar.find_path(start_node, end_node, grid)
                if path:
                    print(f"Found path from {start_node.x},{start_node.y} to {end_node.x},{end_node.y}")
                else:
                    print("No path found.")

        elif command.startswith("OBSTACLE"):
            coords = command.split()[1:]
            for c in coords:
                c = c.strip("{}")
                try:
                    x, y = map(int, c.split(","))
                    node = grid.get_node(x, y)
                    if node:
                        grid.add_obstacle(node)
                        print(f"Added obstacle at {x},{y}")
                    else:
                        print(f"Invalid node: {x},{y}")
                except:
                    print("Invalid coordinate:", c)
        else:
            print("Unknown command.")

# --- Start input thread ---
input_ready.set()
input_thread = threading.Thread(target=input_loop)
input_thread.start()

# --- OpenCV Window ---
WINDOW_NAME = "A* Test"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

# --- Main Loop ---
while input_ready.is_set():
    ret, frame = cap.read()
    if not ret:
        break

    frame_height, frame_width = frame.shape[:2]
    frame = grid_util.draw(frame, window_width=frame_width, window_height=frame_height, path=path)
    cv2.imshow(WINDOW_NAME, frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        input_ready.clear()
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Exited cleanly.")
