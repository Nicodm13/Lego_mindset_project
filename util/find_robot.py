import cv2
import numpy as np
import math

def find_robot(frame, grid_overlay=None):
    """
    This entire script is AI genenerated by Claude AI
    Detect the robot's position and orientation using blue (front) and yellow-green (back) square markers
    
    Args:
        frame: Image captured from overhead camera
        grid_overlay: GridOverlay object to determine grid orientation
        
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
    # Blue mask (front marker) - RGB(130, 176, 173) - a teal/cyan color
    lower_blue = np.array([80, 40, 120])
    upper_blue = np.array([95, 100, 200])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Yellow-green mask (back marker) - RGB(255, 255, 198) to RGB(239, 244, 128)
    lower_green = np.array([25, 40, 100]) 
    upper_green = np.array([40, 255, 255])
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