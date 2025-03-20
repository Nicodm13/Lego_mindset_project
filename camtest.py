import os
import cv2
from robot.grid import Grid
from util.grid_util import GridUtil

# This ensures that the webcam loads instantly, otherwise it takes almost a minute...
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
cap = cv2.VideoCapture(0)

# Get initial webcam resolution dynamically
ret, frame = cap.read()
if not ret:
    print("Webcam couldn't be opened.")
    exit()

height, width, _ = frame.shape

# Create Grid and GridUtil
grid = Grid(300, 300, 10)
grid_util = GridUtil(grid)

WINDOW_NAME = "Webcam Feed"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)  # Make resizable
cv2.setMouseCallback(WINDOW_NAME, grid_util.handle_mouse)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_height, frame_width = frame.shape[:2]

    # Draw grid
    frame = grid_util.draw(frame, window_width=frame_width, window_height=frame_height)

    cv2.imshow(WINDOW_NAME, frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

