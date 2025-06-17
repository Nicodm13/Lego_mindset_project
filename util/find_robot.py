import cv2
import numpy as np
import math

def find_robot(frame, grid_overlay=None, hsv_ranges=None):
    """
    Detect the robot's position and orientation using blue (front) and yellow-green (back) square markers
    
    Args:
        frame: Image captured from overhead camera
        grid_overlay: GridOverlay object to determine grid orientation
        hsv_ranges: Optional tuple (lower_blue, upper_blue, lower_green, upper_green) for color detection
        
    Returns:
        tuple: (x, y, orientation_degrees, output_frame) where:
            x, y - center coordinates of the robot in pixels
            orientation_degrees - orientation in degrees (0-360, where 0 is grid North)
            output_frame - visualization frame with robot markers
            
        If robot not found, returns (None, None, None, frame)
    """
    # Create a copy of the frame
    output = frame.copy()
    
    # Convert to HSV color space for better color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # HSV ranges for blue and yellow-green
    if hsv_ranges:
        lower_blue, upper_blue, lower_green, upper_green = hsv_ranges
    else:
        # Default HSV ranges if not provided
        # Blue mask (front marker)
        lower_blue = np.array([85, 20, 100])
        upper_blue = np.array([110, 150, 220])
        
        # Yellow-green mask (back marker)
        lower_green = np.array([25, 40, 100])
        upper_green = np.array([40, 255, 255])
    
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Apply morphological operations to clean up the masks
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours for blue and green markers
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find the centers of the largest blue and green contours
    blue_center = None
    green_center = None
    
    # Process blue contours
    if blue_contours:
        # Find largest blue contour by area
        largest_blue = max(blue_contours, key=cv2.contourArea)
        
        # Only consider contours with sufficient area
        if cv2.contourArea(largest_blue) > 100:
            M = cv2.moments(largest_blue)
            if M["m00"] != 0:
                blue_x = int(M["m10"] / M["m00"])
                blue_y = int(M["m01"] / M["m00"])
                blue_center = (blue_x, blue_y)
                
                # Get bounding rectangle for the square marker
                x, y, w, h = cv2.boundingRect(largest_blue)
                
                # Draw blue marker on output image (using teal color to match marker)
                cv2.rectangle(output, (x, y), (x+w, y+h), (173, 176, 130), 2)
                cv2.drawContours(output, [largest_blue], 0, (173, 176, 130), 2)
                # Mark center point
                cv2.drawMarker(output, blue_center, (173, 176, 130), cv2.MARKER_CROSS, 10, 2)
    
    # Process green contours
    if green_contours:
        # Find largest green contour by area
        largest_green = max(green_contours, key=cv2.contourArea)
        
        # Only consider contours with sufficient area
        if cv2.contourArea(largest_green) > 100:
            M = cv2.moments(largest_green)
            if M["m00"] != 0:
                green_x = int(M["m10"] / M["m00"])
                green_y = int(M["m01"] / M["m00"])
                green_center = (green_x, green_y)
                
                # Get bounding rectangle for the square marker
                x, y, w, h = cv2.boundingRect(largest_green)
                
                # Draw yellow-green marker on output image
                cv2.rectangle(output, (x, y), (x+w, y+h), (128, 244, 239), 2)
                cv2.drawContours(output, [largest_green], 0, (128, 244, 239), 2)
                # Mark center point
                cv2.drawMarker(output, green_center, (128, 244, 239), cv2.MARKER_CROSS, 10, 2)
    
    # If both markers are found, calculate robot position and orientation
    if blue_center and green_center:
        # Robot's center is the midpoint between the two markers
        robot_x = (blue_center[0] + green_center[0]) // 2
        robot_y = (blue_center[1] + green_center[1]) // 2
        
        # Calculate raw orientation (angle between the line from green to blue and the positive x-axis)
        dx = blue_center[0] - green_center[0]
        dy = blue_center[1] - green_center[1]
        
        # Absolute orientation in the frame (where 0 is positive x-axis)
        orientation_rad = math.atan2(-dy, dx)  # Negative dy because y-axis is inverted in images
        absolute_orientation_deg = math.degrees(orientation_rad)
        
        # Convert to 0-360 range
        absolute_orientation_deg = (absolute_orientation_deg + 360) % 360
        
        # Draw robot's center and orientation line
        cv2.circle(output, (robot_x, robot_y), 7, (0, 255, 0), -1)
        cv2.line(output, green_center, blue_center, (0, 255, 0), 2)
        
        # Adjust orientation relative to the grid
        grid_relative_orientation = absolute_orientation_deg
        grid_north_angle = 0
        
        if grid_overlay is not None:
            # Find the grid cell containing the robot
            grid_x = int(robot_x / grid_overlay.cell_width)
            grid_y = int(robot_y / grid_overlay.cell_height)
            
            # Calculate grid North based on the horizontal grid line angle
            # Get the angle of the nearest horizontal grid line above the robot
            grid_north_angle = get_grid_north_angle(grid_overlay, grid_x, grid_y)
            
            # Adjust orientation relative to grid North
            grid_relative_orientation = (absolute_orientation_deg - grid_north_angle) % 360
            
            # Display grid North angle for debugging
            cv2.putText(output, f"Grid North: {grid_north_angle:.1f}°", 
                       (robot_x + 10, robot_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Display orientations
        cv2.putText(output, f"Robot Orientation: {grid_relative_orientation:.1f}°", 
                   (robot_x + 10, robot_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return robot_x, robot_y, grid_relative_orientation, output
    
    return None, None, None, output

def get_grid_north_angle(grid_overlay, grid_x, grid_y):
    """
    Calculate the angle of the grid's North direction based on the grid cell containing the robot.
    North is defined as perpendicular to the horizontal grid line.
    
    Args:
        grid_overlay: GridOverlay object
        grid_x, grid_y: Grid coordinates of the robot
    
    Returns:
        float: Angle in degrees where 0 is the positive x-axis of the frame
    """
    if not hasattr(grid_overlay, 'transformed_horizontal_lines'):
        # If we don't have transformed lines, try to infer angle from the grid's orientation
        # This is a fallback - ideally, the grid overlay should provide transformed lines
        if hasattr(grid_overlay, 'angle'):
            return grid_overlay.angle
        return 0  # Default to 0 degrees if no grid information is available
    
    # Find the nearest horizontal grid line above the robot
    robot_y_in_pixels = grid_y * grid_overlay.cell_height
    
    # Get all horizontal grid lines
    horizontal_lines = grid_overlay.transformed_horizontal_lines
    
    # Find the nearest horizontal line above the robot
    lines_above = [line for line in horizontal_lines 
                  if line[0][1] <= robot_y_in_pixels and line[1][1] <= robot_y_in_pixels]
    
    if not lines_above:
        # If no lines above, use the top grid line
        if horizontal_lines:
            nearest_line = horizontal_lines[0]
        else:
            return 0  # Default if no lines are available
    else:
        # Find the closest line above
        nearest_line = max(lines_above, key=lambda line: line[0][1])
    
    # Calculate the angle of the line (horizontal grid line)
    x1, y1 = nearest_line[0]
    x2, y2 = nearest_line[1]
    
    # Calculate angle (horizontal grid lines run perpendicular to North)
    # North is 90 degrees rotated from the horizontal line angle
    line_angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    north_angle = (line_angle + 90) % 360
    
    return north_angle

def debug_robot_detection():
    """
    Visual debugging tool to show camera feed and detection results in real-time.
    Returns HSV values for blue and yellow-green markers when user exits.
    
    Shows:
    - Live camera feed
    - HSV thresholds for blue and yellow-green markers
    - Detected markers with contour outlines
    - Robot position and orientation when detected
    
    Interactive controls:
    - Press 'q' to quit and return current HSV values
    - Press 's' to save current HSV values and exit
    - Use trackbars to adjust HSV threshold values
    - Left-click on a blue marker to auto-adjust blue HSV thresholds
    - Right-click on a yellow-green marker to auto-adjust green HSV thresholds
    
    Returns:
        tuple: (lower_blue, upper_blue, lower_green, upper_green) - numpy arrays
               containing the HSV ranges for blue and green markers
    """
    print("Starting robot detection debug window...")
    print("Press 'q' to exit and use current HSV values")
    print("Press 's' to save HSV values and exit immediately")
    print("Left-click on the blue marker to set blue HSV thresholds")
    print("Right-click on the yellow-green marker to set green HSV thresholds")
    
    # Initialize the camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        # Return default values if camera fails
        return (
            np.array([85, 20, 100]),
            np.array([110, 150, 220]),
            np.array([25, 40, 100]),
            np.array([40, 255, 255])
        )
    
    # Create a window for the trackbars
    cv2.namedWindow('HSV Thresholds')
    cv2.namedWindow('Robot Detection Debug')
    
    # Shared variables for mouse callback
    global_hsv = None
    latest_blue_contours = []
    latest_green_contours = []
    
    # Create trackbars for HSV values
    # Blue marker trackbars
    cv2.createTrackbar('Blue H Min', 'HSV Thresholds', 85, 179, lambda x: None)
    cv2.createTrackbar('Blue H Max', 'HSV Thresholds', 110, 179, lambda x: None)
    cv2.createTrackbar('Blue S Min', 'HSV Thresholds', 20, 255, lambda x: None)
    cv2.createTrackbar('Blue S Max', 'HSV Thresholds', 150, 255, lambda x: None)
    cv2.createTrackbar('Blue V Min', 'HSV Thresholds', 100, 255, lambda x: None)
    cv2.createTrackbar('Blue V Max', 'HSV Thresholds', 220, 255, lambda x: None)
    
    # Yellow-green marker trackbars
    cv2.createTrackbar('Green H Min', 'HSV Thresholds', 25, 179, lambda x: None)
    cv2.createTrackbar('Green H Max', 'HSV Thresholds', 40, 179, lambda x: None)
    cv2.createTrackbar('Green S Min', 'HSV Thresholds', 40, 255, lambda x: None)
    cv2.createTrackbar('Green S Max', 'HSV Thresholds', 255, 255, lambda x: None)
    cv2.createTrackbar('Green V Min', 'HSV Thresholds', 100, 255, lambda x: None)
    cv2.createTrackbar('Green V Max', 'HSV Thresholds', 255, 255, lambda x: None)
    
    def mouse_callback(event, x, y, flags, param):
        nonlocal global_hsv, latest_blue_contours, latest_green_contours
        
        # Check if HSV image is available
        if global_hsv is None:
            return
            
        # Get the height of the main frame to check if click is in the top half
        frame_height = global_hsv.shape[0]
        
        # Ensure the click is within the top half (main frame, not mask area)
        if y >= frame_height:
            print("Please click on the main camera view (top half of the window)")
            return
        
        # Check if clicked point is inside any detected blue contour
        inside_blue = False
        if latest_blue_contours:
            for contour in latest_blue_contours:
                if cv2.pointPolygonTest(contour, (x, y), False) >= 0:
                    inside_blue = True
                    h, s, v = global_hsv[y, x]
                    print(f"\nClicked on an already detected blue marker!")
                    print(f"HSV at click: ({h}, {s}, {v})")
                    print("No changes made to thresholds.")
                    break
        
        # Check if clicked point is inside any detected green contour
        inside_green = False
        if latest_green_contours:
            for contour in latest_green_contours:
                if cv2.pointPolygonTest(contour, (x, y), False) >= 0:
                    inside_green = True
                    h, s, v = global_hsv[y, x]
                    print(f"\nClicked on an already detected green marker!")
                    print(f"HSV at click: ({h}, {s}, {v})")
                    print("No changes made to thresholds.")
                    break
        
        # Left button click for blue marker
        if event == cv2.EVENT_LBUTTONDOWN and not inside_blue and not inside_green:
            # Get HSV value at clicked point
            h, s, v = global_hsv[y, x]
            
            # Create a threshold range around the clicked value
            h_margin = 10  # Adjust as needed
            s_margin = 30
            v_margin = 50
            
            h_min = max(0, h - h_margin)
            h_max = min(179, h + h_margin)
            s_min = max(0, s - s_margin)
            s_max = min(255, s + s_margin)
            v_min = max(0, v - v_margin)
            v_max = min(255, v + v_margin)
            
            # Update trackbar values for blue
            cv2.setTrackbarPos('Blue H Min', 'HSV Thresholds', h_min)
            cv2.setTrackbarPos('Blue H Max', 'HSV Thresholds', h_max)
            cv2.setTrackbarPos('Blue S Min', 'HSV Thresholds', s_min)
            cv2.setTrackbarPos('Blue S Max', 'HSV Thresholds', s_max)
            cv2.setTrackbarPos('Blue V Min', 'HSV Thresholds', v_min)
            cv2.setTrackbarPos('Blue V Max', 'HSV Thresholds', v_max)
            
            print(f"\nBlue marker HSV at click: ({h}, {s}, {v})")
            print(f"Set blue HSV range to: [{h_min}-{h_max}, {s_min}-{s_max}, {v_min}-{v_max}]")
            print(f"lower_blue = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"upper_blue = np.array([{h_max}, {s_max}, {v_max}])")
            
        # Right button click for green marker
        elif event == cv2.EVENT_RBUTTONDOWN and not inside_blue and not inside_green:
            # Get HSV value at clicked point
            h, s, v = global_hsv[y, x]
            
            # Create a threshold range around the clicked value
            h_margin = 10  # Adjust as needed
            s_margin = 30
            v_margin = 50
            
            h_min = max(0, h - h_margin)
            h_max = min(179, h + h_margin)
            s_min = max(0, s - s_margin)
            s_max = min(255, s + s_margin)
            v_min = max(0, v - v_margin)
            v_max = min(255, v + v_margin)
            
            # Update trackbar values for green
            cv2.setTrackbarPos('Green H Min', 'HSV Thresholds', h_min)
            cv2.setTrackbarPos('Green H Max', 'HSV Thresholds', h_max)
            cv2.setTrackbarPos('Green S Min', 'HSV Thresholds', s_min)
            cv2.setTrackbarPos('Green S Max', 'HSV Thresholds', s_max)
            cv2.setTrackbarPos('Green V Min', 'HSV Thresholds', v_min)
            cv2.setTrackbarPos('Green V Max', 'HSV Thresholds', v_max)
            
            print(f"\nGreen marker HSV at click: ({h}, {s}, {v})")
            print(f"Set green HSV range to: [{h_min}-{h_max}, {s_min}-{s_max}, {v_min}-{v_max}]")
            print(f"lower_green = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"upper_green = np.array([{h_max}, {s_max}, {v_max}])")
    
    # Set mouse callback
    cv2.setMouseCallback('Robot Detection Debug', mouse_callback)
    
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break
        
        # Create display output
        debug_frame = frame.copy()
        
        # Get current trackbar values
        blue_h_min = cv2.getTrackbarPos('Blue H Min', 'HSV Thresholds')
        blue_h_max = cv2.getTrackbarPos('Blue H Max', 'HSV Thresholds')
        blue_s_min = cv2.getTrackbarPos('Blue S Min', 'HSV Thresholds')
        blue_s_max = cv2.getTrackbarPos('Blue S Max', 'HSV Thresholds')
        blue_v_min = cv2.getTrackbarPos('Blue V Min', 'HSV Thresholds')
        blue_v_max = cv2.getTrackbarPos('Blue V Max', 'HSV Thresholds')
        
        green_h_min = cv2.getTrackbarPos('Green H Min', 'HSV Thresholds')
        green_h_max = cv2.getTrackbarPos('Green H Max', 'HSV Thresholds')
        green_s_min = cv2.getTrackbarPos('Green S Min', 'HSV Thresholds')
        green_s_max = cv2.getTrackbarPos('Green S Max', 'HSV Thresholds')
        green_v_min = cv2.getTrackbarPos('Green V Min', 'HSV Thresholds')
        green_v_max = cv2.getTrackbarPos('Green V Max', 'HSV Thresholds')
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Update global HSV for mouse callback
        global_hsv = hsv
        
        # Create masks based on trackbar values
        lower_blue = np.array([blue_h_min, blue_s_min, blue_v_min])
        upper_blue = np.array([blue_h_max, blue_s_max, blue_v_max])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        lower_green = np.array([green_h_min, green_s_min, green_v_min])
        upper_green = np.array([green_h_max, green_s_max, green_v_max])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Update contours for mouse callback
        latest_blue_contours = blue_contours
        latest_green_contours = green_contours
        
        # Prepare visualization masks (convert to BGR for display)
        blue_mask_vis = cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR)
        green_mask_vis = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
        
        # Process blue contours
        blue_center = None
        if blue_contours:
            # Find largest blue contour by area
            largest_blue = max(blue_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_blue)
            
            # Only consider contours with sufficient area
            if area > 50:  # Lower threshold for debugging
                M = cv2.moments(largest_blue)
                if M["m00"] != 0:
                    blue_x = int(M["m10"] / M["m00"])
                    blue_y = int(M["m01"] / M["m00"])
                    blue_center = (blue_x, blue_y)
                    
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(largest_blue)
                    
                    # Draw on debug frame
                    cv2.rectangle(debug_frame, (x, y), (x+w, y+h), (173, 176, 130), 2)
                    cv2.drawMarker(debug_frame, blue_center, (173, 176, 130), cv2.MARKER_CROSS, 10, 2)
                    cv2.putText(debug_frame, f"Blue: {area:.0f}px²", (x, y-5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (173, 176, 130), 2)
        
        # Process green contours
        green_center = None
        if green_contours:
            # Find largest green contour by area
            largest_green = max(green_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_green)
            
            # Only consider contours with sufficient area
            if area > 50:  # Lower threshold for debugging
                M = cv2.moments(largest_green)
                if M["m00"] != 0:
                    green_x = int(M["m10"] / M["m00"])
                    green_y = int(M["m01"] / M["m00"])
                    green_center = (green_x, green_y)
                    
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(largest_green)
                    
                    # Draw on debug frame
                    cv2.rectangle(debug_frame, (x, y), (x+w, y+h), (128, 244, 239), 2)
                    cv2.drawMarker(debug_frame, green_center, (128, 244, 239), cv2.MARKER_CROSS, 10, 2)
                    cv2.putText(debug_frame, f"Green: {area:.0f}px²", (x, y-5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 244, 239), 2)
        
        # If both markers found, calculate robot position and orientation
        if blue_center and green_center:
            # Calculate robot center
            robot_x = (blue_center[0] + green_center[0]) // 2
            robot_y = (blue_center[1] + green_center[1]) // 2
            
            # Calculate orientation
            dx = blue_center[0] - green_center[0]
            dy = blue_center[1] - green_center[1]
            orientation_rad = math.atan2(-dy, dx)
            orientation_deg = math.degrees(orientation_rad)
            orientation_deg = (orientation_deg + 360) % 360
            
            # Draw robot center and orientation line
            cv2.circle(debug_frame, (robot_x, robot_y), 7, (0, 255, 0), -1)
            cv2.line(debug_frame, green_center, blue_center, (0, 255, 0), 2)
            cv2.putText(debug_frame, f"Robot @ ({robot_x}, {robot_y})", (robot_x + 10, robot_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_frame, f"Angle: {orientation_deg:.1f}°", (robot_x + 10, robot_y + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # If robot not detected, show status
            cv2.putText(debug_frame, "Robot not detected", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Show HSV threshold values on debug frame
        cv2.putText(debug_frame, f"Blue HSV: [{blue_h_min}-{blue_h_max}, {blue_s_min}-{blue_s_max}, {blue_v_min}-{blue_v_max}]", 
                    (10, frame.shape[0]-80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame, f"Green HSV: [{green_h_min}-{green_h_max}, {green_s_min}-{green_s_max}, {green_v_min}-{green_v_max}]", 
                    (10, frame.shape[0]-60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame, "Left-click: set blue HSV | Right-click: set green HSV", 
                    (10, frame.shape[0]-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame, "Press 'q' to quit, 's' to save current HSV values and exit", 
                    (10, frame.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Create combined display (original + masks + debug)
        # Resize masks for display
        h, w = frame.shape[:2]
        mask_h = h // 2
        mask_w = w // 2
        blue_mask_small = cv2.resize(blue_mask_vis, (mask_w, mask_h))
        green_mask_small = cv2.resize(green_mask_vis, (mask_w, mask_h))
        
        # Combine masks side by side
        masks_combined = np.hstack((blue_mask_small, green_mask_small))
        
        # Add titles to masks
        cv2.putText(masks_combined, "Blue Mask", (10, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(masks_combined, "Green Mask", (mask_w + 10, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Stack original debug frame on top and masks on bottom
        top_row = debug_frame
        bottom_row = masks_combined
        
        # Create black padding if needed to make bottom row same width as top
        if bottom_row.shape[1] != top_row.shape[1]:
            padding = np.zeros((bottom_row.shape[0], top_row.shape[1] - bottom_row.shape[1], 3), dtype=np.uint8)
            bottom_row = np.hstack((bottom_row, padding))
        
        # Stack rows vertically
        display = np.vstack((top_row, bottom_row))
        
        # Show the display
        cv2.imshow('Robot Detection Debug', display)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('s'):
            # Save the current HSV values
            print("\n=== Final HSV Values ===")
            print(f"Blue HSV range: [{blue_h_min}-{blue_h_max}, {blue_s_min}-{blue_s_max}, {blue_v_min}-{blue_v_max}]")
            print(f"Green HSV range: [{green_h_min}-{green_h_max}, {green_s_min}-{green_s_max}, {green_v_min}-{green_v_max}]")
            print(f"lower_blue = np.array([{blue_h_min}, {blue_s_min}, {blue_v_min}])")
            print(f"upper_blue = np.array([{blue_h_max}, {blue_s_max}, {blue_v_max}])")
            print(f"lower_green = np.array([{green_h_min}, {green_s_min}, {green_v_min}])")
            print(f"upper_green = np.array([{green_h_max}, {green_s_max}, {green_v_max}])")
            break
    
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    print("HSV calibration completed")
    
    # Return the final HSV values
    return (
        np.array([blue_h_min, blue_s_min, blue_v_min]),
        np.array([blue_h_max, blue_s_max, blue_v_max]),
        np.array([green_h_min, green_s_min, green_v_min]),
        np.array([green_h_max, green_s_max, green_v_max])
    )