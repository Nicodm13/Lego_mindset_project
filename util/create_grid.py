import math

import cv2
import numpy as np

# === CONFIGURABLE VALUES ===
grid_rows = 18
grid_cols = 26
handle_size = 10

# Polygon corners
corners = [(100, 100), (300, 100), (300, 300), (100, 300)]

# State variables
dragging_point = -1


def draw_polygon(frame, polygon_corners):
    """
    Draw the polygon, its warped grid, and the draggable corner handles.

    Parameters:
    - frame: the image frame to draw on
    - pts: list of (x, y) tuples representing the polygon's vertices
    """
    # Draw polygon edges
    corner_array = np.array(polygon_corners, np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [corner_array], isClosed=True, color=(0, 255, 0), thickness=2)

    # Draw warped grid within the polygon using perspective transform
    src = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    dst = np.float32(polygon_corners[:4])  # Only use the first 4 points
    if len(dst) == 4:
        matrix = cv2.getPerspectiveTransform(src * 100, dst)
        for i in range(1, grid_cols):
            alpha = i / grid_cols
            p1 = (1 - alpha) * src[0] + alpha * src[1]
            p2 = (1 - alpha) * src[3] + alpha * src[2]
            grid_line = np.float32([p1, p2]) * 100  # scale unit square to size 100 for simplicity
            warped_line = cv2.perspectiveTransform(grid_line.reshape(-1, 1, 2), matrix)
            pt1, pt2 = warped_line[0][0], warped_line[1][0]
            cv2.line(frame, tuple(pt1.astype(int)), tuple(pt2.astype(int)), (0, 255, 0), 1)

        for j in range(1, grid_rows):
            beta = j / grid_rows
            p1 = (1 - beta) * src[0] + beta * src[3]
            p2 = (1 - beta) * src[1] + beta * src[2]
            grid_line = np.float32([p1, p2]) * 100
            warped_line = cv2.perspectiveTransform(grid_line.reshape(-1, 1, 2), matrix)
            pt1, pt2 = warped_line[0][0], warped_line[1][0]
            cv2.line(frame, tuple(pt1.astype(int)), tuple(pt2.astype(int)), (0, 255, 0), 1)

    # Draw red square handles on each corner point
    for px, py in polygon_corners:
        cv2.rectangle(frame, (px - handle_size, py - handle_size),
                      (px + handle_size, py + handle_size), (0, 0, 255), -1)

    return matrix, src * 100

def get_point_index(mx, my):  # determines which handle which is being clicked
    for i, (px, py) in enumerate(corners):
        if px - handle_size <= mx <= px + handle_size and py - handle_size <= my <= py + handle_size:
            return i
    return -1  # if no handle is clicked


def get_coordinate_from_pixel(mx, my, matrix, src):
    """
    Converts pixel coordinates to grid coordinates.

    Parameters:
    - mx: x coordinate of the pixel
    - my: y coordinate of the pixel
    - matrix: perspective transformation matrix
    - src: source points used for perspective transformation (unused in this function)

    Returns:
    - (gx, gy): grid coordinates
    """
    # Calculate the inverse of the perspective transformation matrix
    inv_matrix = np.linalg.inv(matrix)
    pixel_point = np.array([[mx, my]], dtype=np.float32).reshape(-1, 1, 2)
    grid_point = cv2.perspectiveTransform(pixel_point, inv_matrix)
    gx, gy = grid_point[0][0]

    # Calculate grid cell indices
    cell_x = int(gx * grid_cols / 100)
    cell_y = int(gy * grid_rows / 100)

    # Clamp values to ensure they are within valid grid indices
    cell_x = max(0, min(cell_x, grid_cols - 1))
    cell_y = max(0, min(cell_y, grid_rows - 1))

    return (cell_x, cell_y)


def mouse_events(event, mx, my, flags, param):
    global dragging_point, corners, matrix, src

    if event == cv2.EVENT_LBUTTONDOWN:
        dragging_point = get_point_index(mx, my)
        if dragging_point == -1:
            gx, gy = get_coordinate_from_pixel(mx, my, matrix, src)
            print(f"Grid coordinates: ({gx}, {gy})")

    elif event == cv2.EVENT_MOUSEMOVE and dragging_point != -1:
        corners[dragging_point] = (mx, my)  # moves the handle

    elif event == cv2.EVENT_LBUTTONUP:
        dragging_point = -1  # stops moving the handle

    # elif event == cv2.EVENT_RBUTTONDOWN:
    #     # Right click to add/remove point
    #     index = get_point_index(mx, my)
    #     if index != -1 and len(corners) > 3:
    #         corners.pop(index)
    #     else:
    #         corners.append((mx, my))


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cv2.namedWindow("Adjustable Grid")
cv2.setMouseCallback("Adjustable Grid", mouse_events)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    matrix, src = draw_polygon(frame, corners)
    cv2.imshow("Adjustable Grid", frame)

    key = cv2.waitKey(1)
    if key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()