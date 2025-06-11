# util/path_visualizer.py

import cv2
import numpy as np

def draw_astar_path(frame, path, grid_overlay, color=(255, 0, 0), thickness=2):
    """
    Draws the given A* path on the frame using grid_overlay's perspective matrix.
    
    :param frame: The OpenCV frame to draw on
    :param path: A list of Node objects representing the path
    :param grid_overlay: The GridOverlay instance used for perspective transformation
    :param color: BGR color tuple
    :param thickness: Line thickness
    :return: The modified frame
    """
    if not path or grid_overlay.matrix is None:
        return frame

    # Convert grid coordinates to normalized space and then to screen space
    norm_points = [
        [(node.x + 0.5) / grid_overlay.grid_cols * 100,
         (node.y + 0.5) / grid_overlay.grid_rows * 100]
        for node in path
    ]
    points = np.array(norm_points, dtype=np.float32).reshape(-1, 1, 2)
    transformed = cv2.perspectiveTransform(points, grid_overlay.matrix).astype(int)

    # Draw lines between each pair of points
    for i in range(len(transformed) - 1):
        p1 = tuple(transformed[i][0])
        p2 = tuple(transformed[i + 1][0])
        cv2.line(frame, p1, p2, color, thickness)

    return frame
