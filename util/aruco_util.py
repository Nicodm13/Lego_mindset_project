import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

def get_robot_position_and_angle(frame, grid_overlay, marker_id=0):
    """
    Detects the ArUco marker and returns robot's grid position and orientation.
    - frame: input image
    - grid_overlay: GridOverlay instance
    - marker_id: ArUco ID to track (default = 0)
    Returns: (grid_x, grid_y), angle_deg, annotated_frame
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT)

    if ids is None or marker_id not in ids:
        return None, None, frame

    idx = np.where(ids == marker_id)[0][0]
    marker_corners = corners[idx][0]

    # Use all corners directly for drawing
    cv2.aruco.drawDetectedMarkers(frame, corners)

    # Get center of marker in pixels
    center = np.mean(marker_corners, axis=0)
    center_x, center_y = int(center[0]), int(center[1])

    # Get robot position in grid
    grid_pos = grid_overlay.get_coordinate_from_pixel(center_x, center_y)

    # Calculate angle (yaw) from top edge
    top_edge = marker_corners[1] - marker_corners[0]
    angle_rad = np.arctan2(top_edge[1], top_edge[0])
    angle_deg = (np.degrees(angle_rad) + 360) % 360  # Normalize 0-360

    # Draw heading direction
    tip = (
        int(center_x + 40 * np.cos(angle_rad)),
        int(center_y + 40 * np.sin(angle_rad))
    )
    cv2.arrowedLine(frame, (center_x, center_y), tip, (255, 0, 0), 2)

    return grid_pos, angle_deg, frame
