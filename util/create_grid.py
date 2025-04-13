import math
import cv2
import numpy as np
import shared_data  # Shared module to store safe and unsafe grid coordinates

# === CONFIGURABLE VALUES ===
width = 180      # Width of the virtual grid (used for scaling)
height = 120     # Height of the virtual grid
scale = 2        # Scaling factor for grid resolution
grid_rows = math.floor(height / scale)  # Number of horizontal grid cells
grid_cols = math.floor(width / scale)   # Number of vertical grid cells
handle_size = 10  # Size of draggable corner boxes for perspective editing

# Polygon corners (used for perspective warping)
corners = [(100, 100), (300, 100), (300, 300), (100, 300)]

# Index of a corner being dragged, -1 means no drag in progress
dragging_point = -1


def draw_polygon(frame):
    """
    Draws the polygon, grid lines, filled safe/unsafe cells, and corner handles on the frame.

    Parameters:
    - frame: The current video frame to draw on

    Returns:
    - matrix: The perspective transformation matrix used for warping
    """
    # Define source and destination points for perspective transform
    src = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    dst = np.float32(corners[:4])
    matrix = cv2.getPerspectiveTransform(src * 100, dst)

    # Draw filled cells based on safe and unsafe grid coordinates
    for gx, gy in shared_data.safe_coordinates + shared_data.unsafe_coordinates:
        color = (0, 255, 0) if (gx, gy) in shared_data.safe_coordinates else (0, 0, 255)
        cell = np.float32([
            [gx / grid_cols, gy / grid_rows],
            [(gx + 1) / grid_cols, gy / grid_rows],
            [(gx + 1) / grid_cols, (gy + 1) / grid_rows],
            [gx / grid_cols, (gy + 1) / grid_rows]
        ]) * 100
        warped = cv2.perspectiveTransform(cell.reshape(-1, 1, 2), matrix).reshape(-1, 2).astype(int)
        cv2.fillPoly(frame, [warped], color)

    # Draw vertical grid lines
    for i in range(1, grid_cols):
        alpha = i / grid_cols
        p1 = (1 - alpha) * src[0] + alpha * src[1]
        p2 = (1 - alpha) * src[3] + alpha * src[2]
        line = np.float32([p1, p2]) * 100
        warped = cv2.perspectiveTransform(line.reshape(-1, 1, 2), matrix)
        cv2.line(frame, tuple(warped[0][0].astype(int)), tuple(warped[1][0].astype(int)), (0, 255, 0), 1)

    # Draw horizontal grid lines
    for j in range(1, grid_rows):
        beta = j / grid_rows
        p1 = (1 - beta) * src[0] + beta * src[3]
        p2 = (1 - beta) * src[1] + beta * src[2]
        line = np.float32([p1, p2]) * 100
        warped = cv2.perspectiveTransform(line.reshape(-1, 1, 2), matrix)
        cv2.line(frame, tuple(warped[0][0].astype(int)), tuple(warped[1][0].astype(int)), (0, 255, 0), 1)

    # Draw the outer polygon and corner handles
    cv2.polylines(frame, [np.array(corners, np.int32).reshape((-1, 1, 2))], isClosed=True, color=(0, 255, 0), thickness=2)
    for px, py in corners:
        cv2.rectangle(frame, (px - handle_size, py - handle_size), (px + handle_size, py + handle_size), (0, 0, 255), -1)

    return matrix


def get_point_index(mx, my):
    """
    Determines if the mouse click is on a corner handle.

    Returns:
    - Index of the corner if clicked, otherwise -1.
    """
    for i, (px, py) in enumerate(corners):
        if px - handle_size <= mx <= px + handle_size and py - handle_size <= my <= py + handle_size:
            return i
    return -1


def get_coordinate_from_pixel(mx, my, matrix):
    """
    Converts pixel coordinates from the camera feed into grid cell coordinates.

    Returns:
    - (gx, gy): grid cell indices or (-1, -1) if outside bounds.
    """
    inv_matrix = np.linalg.inv(matrix)
    pixel_point = np.array([[mx, my]], dtype=np.float32).reshape(-1, 1, 2)
    grid_point = cv2.perspectiveTransform(pixel_point, inv_matrix)[0][0]
    gx, gy = grid_point

    # Normalize to [0, 100] x [0, 100] space
    if gx < 0 or gx > 100 or gy < 0 or gy > 100:
        return -1, -1

    # Convert to grid indices
    cell_x = int(gx * grid_cols / 100)
    cell_y = int(gy * grid_rows / 100)

    if cell_x < 0 or cell_x >= grid_cols or cell_y < 0 or cell_y >= grid_rows:
        return -1, -1

    return cell_x, cell_y


def mouse_events(event, mx, my, flags, param):
    """
    Handles mouse clicks and drags:
    - Left-click to toggle safe cells (green)
    - Right-click to toggle unsafe cells (red)
    - Drag handles to reshape the warped polygon
    """
    global dragging_point, corners, matrix, src

    # Left-click: mark or unmark safe cell
    if event == cv2.EVENT_LBUTTONDOWN:
        dragging_point = get_point_index(mx, my)
        if dragging_point == -1:
            gx, gy = get_coordinate_from_pixel(mx, my, param['matrix'])
            if (gx, gy) != (-1, -1):
                if (gx, gy) in shared_data.safe_coordinates:
                    shared_data.safe_coordinates.remove((gx, gy))
                    print(f"Removed {(gx, gy)} from safe_coordinates")
                else:
                    shared_data.safe_coordinates.append((gx, gy))
                    print(f"Added {(gx, gy)} to safe_coordinates")
                    if (gx, gy) in shared_data.unsafe_coordinates:
                        shared_data.unsafe_coordinates.remove((gx, gy))
                        print(f"Removed {(gx, gy)} from unsafe_coordinates")

    # Right-click: mark or unmark unsafe cell
    elif event == cv2.EVENT_RBUTTONDOWN:
        dragging_point = get_point_index(mx, my)
        if dragging_point == -1:
            gx, gy = get_coordinate_from_pixel(mx, my, param['matrix'])
            if (gx, gy) != (-1, -1):
                if (gx, gy) in shared_data.unsafe_coordinates:
                    shared_data.unsafe_coordinates.remove((gx, gy))
                    print(f"Removed {(gx, gy)} from unsafe_coordinates")
                else:
                    shared_data.unsafe_coordinates.append((gx, gy))
                    print(f"Added {(gx, gy)} to unsafe_coordinates")
                    if (gx, gy) in shared_data.safe_coordinates:
                        shared_data.safe_coordinates.remove((gx, gy))
                        print(f"Removed {(gx, gy)} from safe_coordinates")

    # Dragging handle
    elif event == cv2.EVENT_MOUSEMOVE and dragging_point != -1:
        corners[dragging_point] = (mx, my)

    # Release handle
    elif event == cv2.EVENT_LBUTTONUP:
        dragging_point = -1


# === Main loop ===
cap = cv2.VideoCapture(0)  # Open the webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cv2.namedWindow("Adjustable Grid")

# Store the current perspective matrix so mouse_events can use it
mouse_param = {'matrix': None}
cv2.setMouseCallback("Adjustable Grid", mouse_events, mouse_param)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Draw the polygon and grid on the live camera feed
    mouse_param['matrix'] = draw_polygon(frame)
    cv2.imshow("Adjustable Grid", frame)

    # Press ESC to exit
    if cv2.waitKey(1) == 27:
        break

# Release resources when finished
cap.release()
cv2.destroyAllWindows()
