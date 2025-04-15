import math

import cv2
import numpy as np

# === CONFIGURABLE VALUES ===
width = 180
height = 120
scale = 2
grid_rows = math.floor(height/scale)  # height of the field multiplied by some constant
grid_cols = math.floor(width/scale)  # width of the field multiplied by some constant
handle_size = 10

# Polygon corners
corners = [(100, 100), (300, 100), (300, 300), (100, 300)]
# State variables
dragging_point = -1

# Lists
safe_coordinates = []
unsafe_coordinates = []


def draw_polygon(frame):
    """
    Draw the polygon, its warped grid, and the draggable corner handles.

    Parameters:
    - frame: the image frame to draw on
    - pts: list of (x, y) tuples representing the polygon's vertices
    """
    src = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    dst = np.float32(corners[:4])
    matrix = cv2.getPerspectiveTransform(src * 100, dst)

    # Draw marked cells
    for gx, gy in safe_coordinates + unsafe_coordinates:
        color = (0, 255, 0) if (gx, gy) in safe_coordinates else (0, 0, 255)
        cell = np.float32([
            [gx / grid_cols, gy / grid_rows],
            [(gx + 1) / grid_cols, gy / grid_rows],
            [(gx + 1) / grid_cols, (gy + 1) / grid_rows],
            [gx / grid_cols, (gy + 1) / grid_rows]
        ]) * 100
        warped = cv2.perspectiveTransform(cell.reshape(-1, 1, 2), matrix).reshape(-1, 2).astype(int)
        cv2.fillPoly(frame, [warped], color)

    # Draw grid lines
    for i in range(1, grid_cols):
        alpha = i / grid_cols
        p1 = (1 - alpha) * src[0] + alpha * src[1]
        p2 = (1 - alpha) * src[3] + alpha * src[2]
        line = np.float32([p1, p2]) * 100
        warped = cv2.perspectiveTransform(line.reshape(-1, 1, 2), matrix)
        cv2.line(frame, tuple(warped[0][0].astype(int)), tuple(warped[1][0].astype(int)), (0, 255, 0), 1)

    for j in range(1, grid_rows):
        beta = j / grid_rows
        p1 = (1 - beta) * src[0] + beta * src[3]
        p2 = (1 - beta) * src[1] + beta * src[2]
        line = np.float32([p1, p2]) * 100
        warped = cv2.perspectiveTransform(line.reshape(-1, 1, 2), matrix)
        cv2.line(frame, tuple(warped[0][0].astype(int)), tuple(warped[1][0].astype(int)), (0, 255, 0), 1)

    # Draw polygon and handles
    cv2.polylines(frame, [np.array(corners, np.int32).reshape((-1, 1, 2))], isClosed=True, color=(0, 255, 0), thickness=2)
    for px, py in corners:
        cv2.rectangle(frame, (px - handle_size, py - handle_size), (px + handle_size, py + handle_size), (0, 0, 255), -1)

    return matrix

def get_point_index(mx, my):  # determines which handle which is being clicked
    for i, (px, py) in enumerate(corners):
        if px - handle_size <= mx <= px + handle_size and py - handle_size <= my <= py + handle_size:
            return i
    return -1  # if no handle is clicked


def get_coordinate_from_pixel(mx, my, matrix):
    """
    Converts pixel coordinates to grid coordinates.

    Parameters:
    - mx: x coordinate of the pixel
    - my: y coordinate of the pixel
    - matrix: perspective transformation matrix
    - src: source points used for perspective transformation

    Returns:
    - (gx, gy): grid coordinates, or (-1, -1) if outside the grid
    """
    # Calculate the inverse of the perspective transformation matrix
    inv_matrix = np.linalg.inv(matrix)
    pixel_point = np.array([[mx, my]], dtype=np.float32).reshape(-1,1,2)
    grid_point = cv2.perspectiveTransform(pixel_point, inv_matrix)[0][0]
    gx, gy = grid_point

    # First check if the point is outside the normalized [0,100]×[0,100] space
    # (since src was scaled by 100 in draw_polygon)
    if gx < 0 or gx > 100 or gy < 0 or gy > 100:
        return -1, -1

    # Now calculate grid cell indices
    cell_x = int(gx * grid_cols / 100)
    cell_y = int(gy * grid_rows / 100)

    # Final bounds check (shouldn't be needed if the first check passes,
    # but added for numerical stability)
    if cell_x < 0 or cell_x >= grid_cols or cell_y < 0 or cell_y >= grid_rows:
        return -1, -1

    return cell_x, cell_y


def mouse_events(event, mx, my, flags, param):
    global dragging_point, corners, matrix, src

    # Mark safe coordinates
    if event == cv2.EVENT_LBUTTONDOWN:
        dragging_point = get_point_index(mx, my)
        if dragging_point == -1:
            gx, gy = get_coordinate_from_pixel(mx, my, param['matrix'])
            if (gx, gy) != (-1, -1):
                if (gx, gy) in safe_coordinates:
                    safe_coordinates.remove((gx, gy))
                    print(f"removed {(gx, gy)} from safe_coordinates")
                else:
                    safe_coordinates.append((gx, gy))
                    print(f"Added {(gx, gy)} to safe_coordinates")
                    if (gx, gy) in unsafe_coordinates:
                        unsafe_coordinates.remove((gx, gy))
                        print(f"removed {(gx, gy)} from safe_coordinates")

    #Mark unsafe coordinates
    elif event == cv2.EVENT_RBUTTONDOWN:
        dragging_point = get_point_index(mx, my)
        if dragging_point == -1:
            gx, gy = get_coordinate_from_pixel(mx, my, param['matrix'])
            if (gx, gy) != (-1, -1):
                if (gx, gy) in unsafe_coordinates:
                    unsafe_coordinates.remove((gx, gy))
                    print(f"removed {(gx, gy)} from unsafe_coordinates")
                else:
                    unsafe_coordinates.append((gx, gy))
                    print(f"Added {(gx, gy)} to unsafe_coordinates")
                    if (gx, gy) in safe_coordinates:
                        safe_coordinates.remove((gx, gy))
                        print(f"removed {(gx, gy)} from safe_coordinates")

    elif event == cv2.EVENT_MOUSEMOVE and dragging_point != -1:
        corners[dragging_point] = (mx, my)  # moves the handle

    elif event == cv2.EVENT_LBUTTONUP:
        dragging_point = -1  # stops moving the handle



cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cv2.namedWindow("Adjustable Grid")

mouse_param = {'matrix': None}
cv2.setMouseCallback("Adjustable Grid", mouse_events, mouse_param)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    mouse_param['matrix'] = draw_polygon(frame)
    cv2.imshow("Adjustable Grid", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()