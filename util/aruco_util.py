import cv2
import numpy as np

from util.find_robot import get_grid_north_angle

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

def get_robot_position_and_angle_old(frame, grid_overlay, marker_id=0):
    """
    Detects the ArUco marker and returns robot's grid position and orientation.
    - frame: input image
    - grid_overlay: GridOverlay instance
    - marker_id: ArUco ID to track (default = 0)
    Returns: (grid_x, grid_y), angle_deg, annotated_frame
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # For OpenCV 4.7.0+
    detector = cv2.aruco.ArucoDetector(ARUCO_DICT)
    corners, ids, rejected = detector.detectMarkers(gray)

    # For older OpenCV versions (if needed):
    # corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT)

    if ids is None or marker_id not in ids:
        return None, None, frame

    idx = np.where(ids == marker_id)[0][0]
    marker_corners = corners[idx][0]

    # Draw marker for visualization
    cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], np.array([ids[idx]]))
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


def get_robot_position_and_angle(frame, grid_overlay, marker_id=0):
    """
    Detects the ArUco marker with enhanced reliability. By running multiple preprocessing techniques.
    Slower than get_robot_position_and_angle_old, but more robust against noise and lighting changes.
    Returns: (grid_x, grid_y), angle_deg_relative_to_grid, annotated_frame
    """
    annotated_frame = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Image preprocessing variations
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(blurred)

    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant = 7
    detector = cv2.aruco.ArucoDetector(ARUCO_DICT, params)

    detection_attempts = [
        (gray, "Original"),
        (blurred, "Blurred"),
        (enhanced, "Enhanced")
    ]

    for img, method_name in detection_attempts:
        corners, ids, _ = detector.detectMarkers(img)

        if ids is not None and marker_id in ids:
            idx = np.where(ids == marker_id)[0][0]
            marker_corners = corners[idx][0]

            # Get center of marker
            center = np.mean(marker_corners, axis=0)
            center_x, center_y = int(center[0]), int(center[1])

            # Get robot position in grid
            grid_pos = grid_overlay.get_coordinate_from_pixel(center_x, center_y)

            # Calculate angle
            top_edge_mid = (marker_corners[0] + marker_corners[1]) / 2
            heading_vector = center - top_edge_mid
            angle_rad = np.arctan2(heading_vector[1], heading_vector[0])
            absolute_angle = (np.degrees(angle_rad) + 360) % 360

            grid_north_angle = get_grid_north_angle(grid_overlay, center_x, center_y)
            angle_deg = (absolute_angle + grid_north_angle + 360) % 360

            # Draw marker outline
            cv2.polylines(annotated_frame, [np.int32(marker_corners)], isClosed=True, color=(0, 255, 255), thickness=2)

            # Draw heading arrow
            tip = (
                int(center_x + 40 * np.cos(angle_rad)),
                int(center_y + 40 * np.sin(angle_rad))
            )
            cv2.arrowedLine(annotated_frame, (center_x, center_y), tip, (255, 0, 0), 2)

            # Draw grid "North" arrow for visual debugging (yellow arrow)
            north_dx = int(40 * np.cos(np.radians(grid_north_angle)))
            north_dy = int(-40 * np.sin(np.radians(grid_north_angle)))
            cv2.arrowedLine(
                annotated_frame,
                (center_x, center_y),
                (center_x + north_dx, center_y + north_dy),
                (0, 255, 255), 2
            )

            # Draw angle text instead of ID
            cv2.putText(
                annotated_frame,
                f"{grid_pos} w/ deg({angle_deg:.1f})°",
                (center_x + 10, center_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
            print(f"Absolute angle: {absolute_angle:.2f}°, Grid North: {grid_north_angle:.2f}°, Relative: {angle_deg:.2f}°")

            return grid_pos, angle_deg, annotated_frame

    # If marker not found, draw fallback message
    cv2.putText(annotated_frame, "No marker detected", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return None, None, annotated_frame