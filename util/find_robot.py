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
    # output_frame = frame.copy()
    output_frame = frame
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
            # Get approximated polygon of the contour
            peri = cv2.arcLength(largest_blue, True)
            approx = cv2.approxPolyDP(largest_blue, 0.04 * peri, True)

            # Check if shape is rectangular (4 vertices)
            # and has square-like aspect ratio
            x, y, w, h = cv2.boundingRect(largest_blue)
            aspect_ratio = float(w) / h
            is_square_like = len(approx) >= 4 and 0.7 < aspect_ratio < 1.3

            # Only proceed if the shape is square-like
            if is_square_like:
                M = cv2.moments(largest_blue)
                if M["m00"] != 0:
                    blue_x = int(M["m10"] / M["m00"])
                    blue_y = int(M["m01"] / M["m00"])
                    blue_center = (blue_x, blue_y)

                    # Get bounding rectangle for the square marker
                    x, y, w, h = cv2.boundingRect(largest_blue)

                    # Draw blue marker on output image (using teal color to match marker)
                    cv2.rectangle(output_frame, (x, y), (x+w, y+h), (173, 176, 130), 2)
                    cv2.drawContours(output_frame, [largest_blue], 0, (173, 176, 130), 2)
                    # Mark center point
                    cv2.drawMarker(output_frame, blue_center, (173, 176, 130), cv2.MARKER_CROSS, 10, 2)

    # Process green contours
    if green_contours:
        # Find largest green contour by area
        largest_green = max(green_contours, key=cv2.contourArea)
        
        # Only consider contours with sufficient area
        if cv2.contourArea(largest_green) > 100:
            # Get approximated polygon of the contour
            peri = cv2.arcLength(largest_green, True)
            approx = cv2.approxPolyDP(largest_green, 0.04 * peri, True)

            # Check if shape is rectangular (4 vertices)
            # and has square-like aspect ratio
            x, y, w, h = cv2.boundingRect(largest_green)
            aspect_ratio = float(w) / h
            is_square_like = len(approx) >= 4 and 0.7 < aspect_ratio < 1.3

            # Only proceed if the shape is square-like
            if is_square_like:
                M = cv2.moments(largest_green)
                if M["m00"] != 0:
                    green_x = int(M["m10"] / M["m00"])
                    green_y = int(M["m01"] / M["m00"])
                    green_center = (green_x, green_y)

                    # Get bounding rectangle for the square marker
                    x, y, w, h = cv2.boundingRect(largest_green)

                    # Draw yellow-green marker on output image
                    cv2.rectangle(output_frame, (x, y), (x+w, y+h), (128, 244, 239), 2)
                    cv2.drawContours(output_frame, [largest_green], 0, (128, 244, 239), 2)
                    # Mark center point
                    cv2.drawMarker(output_frame, green_center, (128, 244, 239), cv2.MARKER_CROSS, 10, 2)

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
        cv2.circle(output_frame, (robot_x, robot_y), 7, (0, 255, 0), -1)
        cv2.line(output_frame, green_center, blue_center, (0, 255, 0), 2)
        
        # Adjust orientation relative to the grid
        grid_relative_orientation = absolute_orientation_deg
        grid_north_angle = 0
        
        if grid_overlay is not None:
            # Find the grid cell containing the robot using the proper conversion method
            grid_x, grid_y = grid_overlay.get_coordinate_from_pixel(robot_x, robot_y, is_robot=True)
            
            # Only proceed if the robot is within a valid grid cell
            if grid_x != -1 and grid_y != -1:
                # Calculate grid North based on the horizontal grid line angle
                grid_north_angle = get_grid_north_angle(grid_overlay, robot_x, robot_y)
                
                # Adjust orientation relative to grid North
                grid_relative_orientation = (absolute_orientation_deg - grid_north_angle) % 360
                
                # Display grid North angle for debugging
                cv2.putText(output_frame, f"Grid North: {grid_north_angle:.1f}°",
                           (robot_x + 10, robot_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                # Add grid cell coordinates for debugging
                cv2.putText(output_frame, f"Grid Cell: ({grid_x}, {grid_y})",
                           (robot_x + 10, robot_y + 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Display orientations
        cv2.putText(output_frame, f"Robot Orientation: {grid_relative_orientation:.1f}°",
                   (robot_x + 10, robot_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return robot_x, robot_y, grid_relative_orientation, output_frame
    
    return None, None, None, output_frame

def get_grid_north_angle(grid_overlay, pixel_x, pixel_y):
    """
    Calculate the angle of the grid's North direction based on the robot's position in the grid.
    North is defined as perpendicular to the horizontal grid line.

    Args:
        grid_overlay: GridOverlay object
        pixel_x, pixel_y: coordinates in pixels

    Returns:
        float: Angle in degrees where 0 is the positive x-axis of the frame
    """
    if grid_overlay.matrix is None:
        return 0

    p1 = np.array([[0.5, 0]], dtype=np.float32).reshape(-1, 1, 2) * 100
    p2 = np.array([[0.5, 1]], dtype=np.float32).reshape(-1, 1, 2) * 100

    warped_p1 = cv2.perspectiveTransform(p1, grid_overlay.matrix)[0][0]
    warped_p2 = cv2.perspectiveTransform(p2, grid_overlay.matrix)[0][0]

    dx = warped_p2[0] - warped_p1[0]
    dy = warped_p2[1] - warped_p1[1]
    angle = (math.degrees(math.atan2(-dy, dx)) + 180) % 360
    return angle

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
    - Press '+' to increase sensitivity (decrease min contour area)
    - Press '-' to decrease sensitivity (increase min contour area)
    
    Returns:
        tuple: (lower_blue, upper_blue, lower_green, upper_green) - numpy arrays
               containing the HSV ranges for blue and green markers
    """

    print("Press 'q' to exit and use current HSV values")
    print("Press 's' to save HSV values and exit immediately")
    print("Left-click on the blue marker to set blue HSV thresholds")
    print("Right-click on the yellow-green marker to set green HSV thresholds")
    print("Press '+' to increase sensitivity (decrease min contour area)")
    print("Press '-' to decrease sensitivity (increase min contour area)")
    print("Starting robot detection debug window...")
    
    # Initialize the camera
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
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
    mouse_pos = (0, 0)  # Track mouse position for HSV display
    min_contour_area = 25  # Lower this threshold for better detection in poor lighting
    
    # Store HSV range values directly (instead of just relying on trackbars)
    # Blue HSV ranges
    blue_h_min, blue_h_max = 85, 110
    blue_s_min, blue_s_max = 20, 150
    blue_v_min, blue_v_max = 100, 220
    
    # Green HSV ranges
    green_h_min, green_h_max = 25, 40
    green_s_min, green_s_max = 40, 255
    green_v_min, green_v_max = 100, 255
    
    # Create trackbars for HSV values
    # Blue marker trackbars
    cv2.createTrackbar('Blue H Min', 'HSV Thresholds', blue_h_min, 179, lambda x: None)
    cv2.createTrackbar('Blue H Max', 'HSV Thresholds', blue_h_max, 179, lambda x: None)
    cv2.createTrackbar('Blue S Min', 'HSV Thresholds', blue_s_min, 255, lambda x: None)
    cv2.createTrackbar('Blue S Max', 'HSV Thresholds', blue_s_max, 255, lambda x: None)
    cv2.createTrackbar('Blue V Min', 'HSV Thresholds', blue_v_min, 255, lambda x: None)
    cv2.createTrackbar('Blue V Max', 'HSV Thresholds', blue_v_max, 255, lambda x: None)
    
    # Yellow-green marker trackbars
    cv2.createTrackbar('Green H Min', 'HSV Thresholds', green_h_min, 179, lambda x: None)
    cv2.createTrackbar('Green H Max', 'HSV Thresholds', green_h_max, 179, lambda x: None)
    cv2.createTrackbar('Green S Min', 'HSV Thresholds', green_s_min, 255, lambda x: None)
    cv2.createTrackbar('Green S Max', 'HSV Thresholds', green_s_max, 255, lambda x: None)
    cv2.createTrackbar('Green V Min', 'HSV Thresholds', green_v_min, 255, lambda x: None)
    cv2.createTrackbar('Green V Max', 'HSV Thresholds', green_v_max, 255, lambda x: None)
    
    # Add kernel size trackbar for morphological operations
    cv2.createTrackbar('Morph Kernel', 'HSV Thresholds', 5, 15, lambda x: None)

    def ensure_valid_range(min_val, max_val, is_hue=False):
        """Helper function to ensure min value is less than or equal to max value"""
        # Clamp values to valid range
        if is_hue:
            # Hue is 0-179 in OpenCV
            min_val = max(0, min(179, min_val))
            max_val = max(0, min(179, max_val))
        else:
            # Saturation and Value are 0-255
            min_val = max(0, min(255, min_val))
            max_val = max(0, min(255, max_val))

        # Ensure min <= max
        if min_val > max_val:
            return max_val, min_val
        return min_val, max_val

    def update_trackbars_from_vars():
        """Update all trackbar positions from the stored variables"""
        # Update blue trackbar positions
        cv2.setTrackbarPos('Blue H Min', 'HSV Thresholds', blue_h_min)
        cv2.setTrackbarPos('Blue H Max', 'HSV Thresholds', blue_h_max)
        cv2.setTrackbarPos('Blue S Min', 'HSV Thresholds', blue_s_min)
        cv2.setTrackbarPos('Blue S Max', 'HSV Thresholds', blue_s_max)
        cv2.setTrackbarPos('Blue V Min', 'HSV Thresholds', blue_v_min)
        cv2.setTrackbarPos('Blue V Max', 'HSV Thresholds', blue_v_max)
        
        # Update green trackbar positions
        cv2.setTrackbarPos('Green H Min', 'HSV Thresholds', green_h_min)
        cv2.setTrackbarPos('Green H Max', 'HSV Thresholds', green_h_max)
        cv2.setTrackbarPos('Green S Min', 'HSV Thresholds', green_s_min)
        cv2.setTrackbarPos('Green S Max', 'HSV Thresholds', green_s_max)
        cv2.setTrackbarPos('Green V Min', 'HSV Thresholds', green_v_min)
        cv2.setTrackbarPos('Green V Max', 'HSV Thresholds', green_v_max)

    def mouse_callback(event, x, y, flags, param):
        nonlocal global_hsv, latest_blue_contours, latest_green_contours, mouse_pos
        nonlocal blue_h_min, blue_h_max, blue_s_min, blue_s_max, blue_v_min, blue_v_max
        nonlocal green_h_min, green_h_max, green_s_min, green_s_max, green_v_min, green_v_max
        
        # Update mouse position for HSV display
        mouse_pos = (x, y)
        
        # Check if HSV image is available
        if global_hsv is None:
            return
            
        # Get the height of the main frame to check if click is in the top half
        frame_height = global_hsv.shape[0]
        
        # Ensure the click is within the top half (main frame, not mask area)
        if y >= frame_height:
            print("Please click on the main camera view (top half of the window)")
            return
        
        # Left button click for blue marker
        if event == cv2.EVENT_LBUTTONDOWN:
            # Get HSV value at clicked point
            h, s, v = global_hsv[y, x]
            
            # Create a threshold range around the clicked value with wider margins
            h_margin = 20  # Increased margin for better detection
            s_margin = 30
            v_margin = 50
            
            # Convert to Python int to prevent overflow during arithmetic operations
            h, s, v = int(h), int(s), int(v)
            
            # Calculate min/max values directly with proper clamping
            h_min = max(0, h - h_margin)
            h_max = min(179, h + h_margin)  # Hue is 0-179 in OpenCV
            s_min = max(0, s - s_margin)
            s_max = min(255, s + s_margin)  # Saturation is 0-255
            v_min = max(0, v - v_margin)
            v_max = min(255, v + v_margin)  # Value is 0-255
            
            # Ensure max is always >= min for each channel
            if h_max < h_min:
                h_min, h_max = h_max, h_min
            if s_max < s_min:
                s_min, s_max = s_max, s_min
            if v_max < v_min:
                v_min, v_max = v_max, v_min
                
            # Update our stored blue values
            blue_h_min, blue_h_max = h_min, h_max
            blue_s_min, blue_s_max = s_min, s_max
            blue_v_min, blue_v_max = v_min, v_max
            
            # Update trackbar values
            update_trackbars_from_vars()
            
            print(f"\nBlue marker HSV at click: ({h}, {s}, {v})")
            print(f"Set blue HSV range to: [{h_min}-{h_max}, {s_min}-{s_max}, {v_min}-{v_max}]")
            print(f"lower_blue = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"upper_blue = np.array([{h_max}, {s_max}, {v_max}])")

        # Right button click for green marker
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Get HSV value at clicked point
            h, s, v = global_hsv[y, x]
            
            # Create a threshold range around the clicked value with wider margins
            h_margin = 20
            s_margin = 30
            v_margin = 50
            
            # Convert to Python int to prevent overflow during arithmetic operations
            h, s, v = int(h), int(s), int(v)
            
            # Calculate min/max values directly with proper clamping
            h_min = max(0, h - h_margin)
            h_max = min(179, h + h_margin)  # Hue is 0-179 in OpenCV
            s_min = max(0, s - s_margin)
            s_max = min(255, s + s_margin)  # Saturation is 0-255
            v_min = max(0, v - v_margin)
            v_max = min(255, v + v_margin)  # Value is 0-255
            
            # Ensure max is always >= min for each channel
            if h_max < h_min:
                h_min, h_max = h_max, h_min
            if s_max < s_min:
                s_min, s_max = s_max, s_min
            if v_max < v_min:
                v_min, v_max = v_max, v_min
                
            # Update our stored green values
            green_h_min, green_h_max = h_min, h_max
            green_s_min, green_s_max = s_min, s_max
            green_v_min, green_v_max = v_min, v_max
            
            # Update trackbar values
            update_trackbars_from_vars()
            
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
        
        # Get current trackbar values and update our stored variables
        # This allows manual adjustment via trackbars
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
        
        # Get morphological kernel size
        kernel_size = cv2.getTrackbarPos('Morph Kernel', 'HSV Thresholds')
        if kernel_size < 1:  # Ensure kernel size is at least 1
            kernel_size = 1
            
        # Ensure that min values are always less than or equal to max values
        # If not, swap them and update the trackbars
        if blue_h_min > blue_h_max:
            blue_h_min, blue_h_max = blue_h_max, blue_h_min
            cv2.setTrackbarPos('Blue H Min', 'HSV Thresholds', blue_h_min)
            cv2.setTrackbarPos('Blue H Max', 'HSV Thresholds', blue_h_max)
        if blue_s_min > blue_s_max:
            blue_s_min, blue_s_max = blue_s_max, blue_s_min
            cv2.setTrackbarPos('Blue S Min', 'HSV Thresholds', blue_s_min)
            cv2.setTrackbarPos('Blue S Max', 'HSV Thresholds', blue_s_max)
        if blue_v_min > blue_v_max:
            blue_v_min, blue_v_max = blue_v_max, blue_v_min
            cv2.setTrackbarPos('Blue V Min', 'HSV Thresholds', blue_v_min)
            cv2.setTrackbarPos('Blue V Max', 'HSV Thresholds', blue_v_max)
            
        if green_h_min > green_h_max:
            green_h_min, green_h_max = green_h_max, green_h_min
            cv2.setTrackbarPos('Green H Min', 'HSV Thresholds', green_h_min)
            cv2.setTrackbarPos('Green H Max', 'HSV Thresholds', green_h_max)
        if green_s_min > green_s_max:
            green_s_min, green_s_max = green_s_max, green_s_min
            cv2.setTrackbarPos('Green S Min', 'HSV Thresholds', green_s_min)
            cv2.setTrackbarPos('Green S Max', 'HSV Thresholds', green_s_max)
        if green_v_min > green_v_max:
            green_v_min, green_v_max = green_v_max, green_v_min
            cv2.setTrackbarPos('Green V Min', 'HSV Thresholds', green_v_min)
            cv2.setTrackbarPos('Green V Max', 'HSV Thresholds', green_v_max)
        
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
        
        # Apply morphological operations with adjustable kernel
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
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
            if area > min_contour_area:  # Using variable threshold for debugging
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
            if area > min_contour_area:  # Using variable threshold for debugging
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
        
        # Display HSV value at current mouse position
        if 0 <= mouse_pos[0] < frame.shape[1] and 0 <= mouse_pos[1] < frame.shape[0]:
            h, s, v = hsv[mouse_pos[1], mouse_pos[0]]
            cv2.putText(debug_frame, f"HSV at cursor: ({h}, {s}, {v})", 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Show HSV threshold values on debug frame
        cv2.putText(debug_frame, f"Blue HSV: [{blue_h_min}-{blue_h_max}, {blue_s_min}-{blue_s_max}, {blue_v_min}-{blue_v_max}]", 
                    (10, frame.shape[0]-100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame, f"Green HSV: [{green_h_min}-{green_h_max}, {green_s_min}-{green_s_max}, {green_v_min}-{green_v_max}]", 
                    (10, frame.shape[0]-80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame, f"Min Area: {min_contour_area} | Kernel: {kernel_size}x{kernel_size}", 
                    (10, frame.shape[0]-60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame, "Left-click: set blue HSV | Right-click: set green HSV", 
                    (10, frame.shape[0]-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame, "'.'/'-': adjust sensitivity | 'q': quit | 's': save and exit",
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
        elif key == ord('+') or key == ord('.'):
            # Increase detection sensitivity by decreasing the minimum contour area
            min_contour_area = max(5, min_contour_area - 5)
            print(f"Increased sensitivity: min_contour_area = {min_contour_area}")
        elif key == ord('-') or key == ord('_'):
            # Decrease detection sensitivity by increasing the minimum contour area
            min_contour_area = min_contour_area + 5
            print(f"Decreased sensitivity: min_contour_area = {min_contour_area}")
    
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

def draw_robot_detection_overlay(frame, robot_x=None, robot_y=None, orientation=None, blue_center=None, green_center=None, scale=0.5):
    """
    Creates a dedicated visualization frame showing robot detection details
    
    Args:
        frame: Original image frame
        robot_x, robot_y: Robot center coordinates (if detected)
        orientation: Robot orientation in degrees (if detected)
        blue_center: Coordinates of blue marker center (if detected)
        green_center: Coordinates of green marker center (if detected)
        scale: Scale factor for the visualization (default 0.5)
        
    Returns:
        Visualization frame with robot detection overlay
    """
    # Create a clean copy of the frame for visualization
    vis_frame = frame.copy()
    
    # Create background with grid
    h, w = vis_frame.shape[:2]
    grid_spacing = 50
    
    # Draw grid lines
    for x in range(0, w, grid_spacing):
        cv2.line(vis_frame, (x, 0), (x, h), (50, 50, 50), 1)
    for y in range(0, h, grid_spacing):
        cv2.line(vis_frame, (0, y), (w, y), (50, 50, 50), 1)
    
    # Add title and border
    title_bar = np.ones((40, w, 3), dtype=np.uint8) * 50
    cv2.putText(title_bar, "Robot Detection Visualization", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    vis_frame = np.vstack([title_bar, vis_frame])
    
    # Draw markers and robot if detected
    if blue_center and green_center:
        # Draw blue marker with rectangle
        cv2.circle(vis_frame, blue_center, 10, (173, 176, 130), -1)  # Fill circle
        cv2.rectangle(vis_frame, 
                     (blue_center[0] - 15, blue_center[1] - 15), 
                     (blue_center[0] + 15, blue_center[1] + 15), 
                     (173, 176, 130), 2)
        cv2.putText(vis_frame, "Blue", (blue_center[0] + 20, blue_center[1]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (173, 176, 130), 2)
        
        # Draw green marker with rectangle
        cv2.circle(vis_frame, green_center, 10, (128, 244, 239), -1)  # Fill circle
        cv2.rectangle(vis_frame, 
                     (green_center[0] - 15, green_center[1] - 15), 
                     (green_center[0] + 15, green_center[1] + 15), 
                     (128, 244, 239), 2)
        cv2.putText(vis_frame, "Green", (green_center[0] + 20, green_center[1]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 244, 239), 2)
        
        # Draw robot center and orientation line
        if robot_x is not None and robot_y is not None:
            robot_center = (robot_x, robot_y)
            cv2.circle(vis_frame, robot_center, 7, (0, 255, 0), -1)  # Robot center
            
            # Draw line between markers
            cv2.line(vis_frame, green_center, blue_center, (0, 255, 0), 2)
            
            # Draw orientation direction with arrow
            if orientation is not None:
                # Calculate end point of orientation arrow (longer than the robot)
                length = 100  # Length of the direction arrow
                end_x = int(robot_x + length * math.cos(math.radians(orientation)))
                end_y = int(robot_y - length * math.sin(math.radians(orientation)))
                
                # Draw the arrow
                cv2.arrowedLine(vis_frame, robot_center, (end_x, end_y), (0, 0, 255), 2, tipLength=0.2)
                
                # Display angle and position text
                cv2.putText(vis_frame, f"Position: ({robot_x}, {robot_y})", (10, vis_frame.shape[0] - 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(vis_frame, f"Orientation: {orientation:.1f}°", (10, vis_frame.shape[0] - 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    else:
        # If robot not detected
        cv2.putText(vis_frame, "Robot not detected", (10, vis_frame.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # Resize visualization if needed
    if scale != 1.0:
        new_w = int(vis_frame.shape[1] * scale)
        new_h = int(vis_frame.shape[0] * scale)
        vis_frame = cv2.resize(vis_frame, (new_w, new_h))
    
    return vis_frame