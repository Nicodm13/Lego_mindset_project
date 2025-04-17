import cv2
from ultralytics import YOLO


def find_ping_pong_balls(frame, grid_overlay, model_path='models/model1.pt'):
    """
    Detects white and orange ping pong balls in a frame and maps them to grid coordinates.

    Args:
        frame: OpenCV image frame
        grid_overlay: GridOverlay instance to map pixel coordinates to grid
        model_path: Path to the YOLO model

    Returns:
        dict: {
            'white_balls': {
                'pixels': [(x, y), ...],  # Pixel coordinates
                'grid': [(gx, gy), ...]   # Grid coordinates
            },
            'orange_balls': {
                'pixels': [(x, y), ...],
                'grid': [(gx, gy), ...]
            }
        }
    """

    import os
    # Get the absolute path to the model file
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    model_abs_path = os.path.join(base_dir, model_path)

    # Load the YOLO model with the absolute path
    model = YOLO(model_abs_path)

    # Process the frame
    results = model(frame, imgsz=640)

    white_balls_pixels = []
    orange_balls_pixels = []

    # Process detection results
    for result in results:
        boxes = result.boxes

        for box in boxes:
            # Get class ID
            cls_id = int(box.cls.item())

            # Get confidence score
            confidence = box.conf.item()

            # Get box coordinates
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # Calculate center point
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Assuming class 0 is white ball and class 1 is orange ball
            # (adjust based on your actual model classes)
            if cls_id == 0:  # White ball
                white_balls_pixels.append((center_x, center_y))
            elif cls_id == 1:  # Orange ball
                orange_balls_pixels.append((center_x, center_y))

    # Map pixel coordinates to grid coordinates
    white_balls_grid = []
    for x, y in white_balls_pixels:
        gx, gy = grid_overlay.get_coordinate_from_pixel(x, y)
        if gx != -1 and gy != -1:  # Valid grid position
            white_balls_grid.append((gx, gy))

    orange_balls_grid = []
    for x, y in orange_balls_pixels:
        gx, gy = grid_overlay.get_coordinate_from_pixel(x, y)
        if gx != -1 and gy != -1:  # Valid grid position
            orange_balls_grid.append((gx, gy))

    return {
        'white_balls': {
            'pixels': white_balls_pixels,
            'grid': white_balls_grid
        },
        'orange_balls': {
            'pixels': orange_balls_pixels,
            'grid': orange_balls_grid
        }
    }


def draw_ball_detections(frame, ball_data):
    """
    Draw circles for detected ping pong balls on the frame

    Args:
        frame: OpenCV image frame
        ball_data: Dictionary returned by find_ping_pong_balls

    Returns:
        frame: The frame with drawn ball indicators
    """
    # Draw white balls
    for x, y in ball_data['white_balls']['pixels']:
        cv2.circle(frame, (x, y), 10, (255, 255, 255), 2)

    # Draw orange balls
    for x, y in ball_data['orange_balls']['pixels']:
        cv2.circle(frame, (x, y), 10, (0, 165, 255), 2)

    return frame