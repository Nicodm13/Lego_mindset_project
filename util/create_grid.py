import cv2
import numpy as np

# === CONFIGURABLE VALUES ===
grid_rows = 60
grid_cols = 90
handle_size = 10

# Polygon corners
corners = [(100, 100), (300, 100), (300, 300), (100, 300)]

# State variables
dragging_point = -1


def draw_polygon(frame, polygoncorners):
    # Draw polygon edges
    corner_array = np.array(polygoncorners, np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [corner_array], isClosed=True, color=(0, 255, 0), thickness=2) #draws the polygon

    # Draw grid
    bbox = cv2.boundingRect(corner_array) #returns the bounding rectangle of the polygon
    x, y, w, h = bbox
    for i in range(1, grid_cols):
        x_pos = x + i * w // grid_cols
        cv2.line(frame, (x_pos, y), (x_pos, y + h), (255, 255, 255), 1)
    for i in range(1, grid_rows):
        y_pos = y + i * h // grid_rows
        cv2.line(frame, (x, y_pos), (x + w, y_pos), (255, 255, 255), 1)

    # Draw adjustable corner handles
    for px, py in polygoncorners:
        cv2.rectangle(frame, (px - handle_size, py - handle_size),
                      (px + handle_size, py + handle_size), (0, 0, 255), -1)


def get_point_index(mx, my):  # determines which handle which is being clicked
    for i, (px, py) in enumerate(corners):
        if px - handle_size <= mx <= px + handle_size and py - handle_size <= my <= py + handle_size:
            return i
    return -1  # if no handle is clicked


def mouse_events(event, mx, my, flags, param):
    global dragging_point, corners

    if event == cv2.EVENT_LBUTTONDOWN:
        dragging_point = get_point_index(mx, my)

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

    draw_polygon(frame, corners)
    cv2.imshow("Adjustable Grid", frame)

    key = cv2.waitKey(1)
    if key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()