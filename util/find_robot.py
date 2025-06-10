import cv2
import numpy as np
import math

def find_robot(frame):
    """
    Detect the robot's position and orientation using blue (front) and red (back) tape markers
    
    Args:
        frame: Image captured from overhead camera
        
    Returns:
        tuple: (x, y, orientation_degrees) where:
            x, y - center coordinates of the robot in pixels
            orientation_degrees - orientation in degrees (0-360, where 0 is along the positive x-axis)
            
        If robot not found, returns (None, None, None)
    """
    # Create a copy of the frame
    output = frame.copy()
    
    # Convert to HSV color space for better color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # HSV ranges for blue and red
    # Blue mask (front marker)
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Red mask (back marker) - Red wraps around in HSV, so we need two ranges
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    # Apply morphological operations to clean up the masks
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours for blue and red markers
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find the centers of the largest blue and red contours
    blue_center = None
    red_center = None
    
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
                
                # Draw blue marker on output image
                cv2.circle(output, blue_center, 5, (255, 0, 0), -1)
                cv2.drawContours(output, [largest_blue], 0, (255, 0, 0), 2)
    
    # Process red contours
    if red_contours:
        # Find largest red contour by area
        largest_red = max(red_contours, key=cv2.contourArea)
        
        # Only consider contours with sufficient area
        if cv2.contourArea(largest_red) > 100:
            M = cv2.moments(largest_red)
            if M["m00"] != 0:
                red_x = int(M["m10"] / M["m00"])
                red_y = int(M["m01"] / M["m00"])
                red_center = (red_x, red_y)
                
                # Draw red marker on output image
                cv2.circle(output, red_center, 5, (0, 0, 255), -1)
                cv2.drawContours(output, [largest_red], 0, (0, 0, 255), 2)
    
    # If both markers are found, calculate robot position and orientation
    if blue_center and red_center:
        # Robot's center is the midpoint between the two markers
        robot_x = (blue_center[0] + red_center[0]) // 2
        robot_y = (blue_center[1] + red_center[1]) // 2
        
        # Calculate orientation (angle between the line from red to blue and the positive x-axis)
        # Math: tan(θ) = (y2-y1)/(x2-x1), then convert to degrees
        dx = blue_center[0] - red_center[0]
        dy = blue_center[1] - red_center[1]
        
        orientation_rad = math.atan2(-dy, dx)  # Negative dy because y-axis is inverted in images
        orientation_deg = math.degrees(orientation_rad)
        
        # Convert to 0-360 range
        orientation_deg = (orientation_deg + 360) % 360
        
        # Draw robot's center and orientation line
        cv2.circle(output, (robot_x, robot_y), 7, (0, 255, 0), -1)
        cv2.line(output, red_center, blue_center, (0, 255, 0), 2)
        cv2.putText(output, f"Orientation: {orientation_deg:.1f}°", 
                   (robot_x + 10, robot_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return robot_x, robot_y, orientation_deg, output
    
    return None, None, None, output
