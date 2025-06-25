import sys
import os
import cv2
import time

# Add robot folder to sys.path before imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

from robot.config import ROBOT_WIDTH, ROBOT_LENGTH
from robot.grid import Grid
from pathfinding.astar import AStar
from util.grid_overlay import GridOverlay
from util.find_balls import find_ping_pong_balls, draw_ball_detections
from util.path_visualizer import draw_astar_path

# --- Grid & Webcam Setup ---
grid = Grid(1800, 1200, 17)

def handle_obstacle_marked(gx, gy):
    node = grid.get_node(gx, gy)
    if node:
        grid.add_obstacle(node)
        print(f"Marked obstacle: ({gx}, {gy})")

def handle_obstacle_unmarked(gx, gy):
    node = grid.get_node(gx, gy)
    if node:
        grid.remove_obstacle(node)
        print(f"Unmarked obstacle: ({gx}, {gy})")

grid_overlay = GridOverlay(
    grid.width, grid.height, grid.density,
    on_mark_obstacle=handle_obstacle_marked,
    on_unmark_obstacle=handle_obstacle_unmarked
)

print("Opening webcam...")
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Wait until it's ready
while not cap.isOpened():
    print("Waiting for camera...")
    time.sleep(0.5)

# Warm-up
for _ in range(5):
    cap.read()
    cv2.waitKey(30)

ret, frame = cap.read()
if not ret:
    print("Webcam couldn't be opened.")
    exit()

# --- OpenCV Window Setup ---
WINDOW_NAME = "Webcam Feed (Test Mode)"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(WINDOW_NAME, grid_overlay.mouse_events)

# --- State ---
start_node = None
visited_balls = set()
tsp_path = []
latest_path = []
detect_balls = False
ball_data = {
    'white_balls': {'pixels': [], 'grid': []},
    'orange_balls': {'pixels': [], 'grid': []}
}

# --- Main Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    original_frame = frame.copy()

    if detect_balls:
        ball_data = find_ping_pong_balls(original_frame, grid_overlay)
        frame = draw_ball_detections(frame, ball_data)

        white_txt = f"White: {len(ball_data['white_balls']['grid'])} balls"
        orange_txt = f"Orange: {len(ball_data['orange_balls']['grid'])} balls"
        cv2.putText(frame, white_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, orange_txt, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

    frame = grid_overlay.draw(frame)

    # Draw path
    if latest_path:
        frame = draw_astar_path(frame, latest_path, grid_overlay)

    cv2.putText(frame, "Press 'R' to reset | 'D' to toggle detection | 'Q' to quit", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    cv2.imshow(WINDOW_NAME, frame)

    # Auto path planning
    all_balls = ball_data['white_balls']['grid'] + ball_data['orange_balls']['grid']
    unvisited = [coord for coord in all_balls if coord not in visited_balls]

    if not tsp_path and unvisited:
        if not start_node:
            if grid_overlay.start_point:
                sx, sy = grid_overlay.start_point
                start_node = grid.get_node(sx, sy)
            else:
                start_node = grid.get_node(0, 0)
                print("Default start node used.")

        closest = AStar.get_closest_nodes(start_node, unvisited, grid, n=3)
        tsp_path.extend(AStar.tsp_brute_force(start_node, closest, grid))

    if tsp_path:
        next_node = tsp_path.pop(0)
        path = AStar.find_path(start_node, next_node, grid, robot_width=ROBOT_WIDTH, robot_length=ROBOT_LENGTH)
        if path:
            latest_path = path
            visited_balls.add((next_node.x, next_node.y))
            start_node = next_node
            print(f"Path to ({next_node.x}, {next_node.y}) found with {len(path)} nodes:")
            for node in path:
                print(f"({node.x}, {node.y})")
        else:
            print("No valid path to next ball, skipping.")

    # --- Key Handling ---
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('d'):
        detect_balls = not detect_balls
        print("Ball detection enabled" if detect_balls else "Ball detection disabled")
    elif key == ord('r'):
        visited_balls.clear()
        tsp_path.clear()
        latest_path.clear()
        start_node = None

        ball_data['white_balls']['pixels'].clear()
        ball_data['white_balls']['grid'].clear()
        ball_data['orange_balls']['pixels'].clear()
        ball_data['orange_balls']['grid'].clear()

        for col in grid.grid:
            for node in col:
                node.is_obstacle = False
        grid_overlay.obstacles.clear()
        grid_overlay.start_point = None

        print("System reset.")

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")
