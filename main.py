import os
import sys
import cv2

# Add robot folder to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

from robot.grid import Grid
from util.grid_util import GridUtil

# --- Webcam Setup ---
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if not ret:
    print("Webcam couldn't be opened.")
    exit()

# --- Grid Setup ---
grid = Grid(width=1800, height=1200, density=4)
grid_util = GridUtil(grid)

# --- OpenCV Window ---
WINDOW_NAME = "Webcam Feed with Grid"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(WINDOW_NAME, grid_util.handle_mouse)  # ← Enable grid button clicking!


print("=== Real-World Distance Calculator ===")
print(f"Grid initialized: {grid.density}x{grid.density} nodes")
print(f"Field size: {grid.width}x{grid.height} units")
print("Use the window to view the grid, and the terminal to enter node coordinates.\n")

# --- Input Loop in Background ---
import threading

def input_loop():
    while True:
        try:
            x1 = input("Start node X (or 'q' to quit): ").strip()
            if x1.lower() == 'q':
                break
            y1 = input("Start node Y: ").strip()
            x2 = input("Target node X: ").strip()
            y2 = input("Target node Y: ").strip()

            node_a = grid.get_node(int(x1), int(y1))
            node_b = grid.get_node(int(x2), int(y2))

            if node_a is None or node_b is None:
                print("Invalid node coordinates. Please enter values within the grid range.")
                continue

            distance = grid.get_grid_distance(node_a, node_b)
            print(f"Distance from ({x1}, {y1}) to ({x2}, {y2}) is {distance:.2f} units.\n")

        except ValueError:
            print("Please enter valid integers for coordinates.\n")
        except KeyboardInterrupt:
            break

    print("Input loop exited.")

# Start input thread
threading.Thread(target=input_loop, daemon=True).start()

# --- Main Webcam Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_height, frame_width = frame.shape[:2]
    frame = grid_util.draw(frame, window_width=frame_width, window_height=frame_height)
    cv2.imshow(WINDOW_NAME, frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")
