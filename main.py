import os
import sys
import cv2
import socket
import threading
from util.find_balls import find_ping_pong_balls, draw_ball_detections

# Add robot folder to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot'))

from robot.grid import Grid
from util.grid_overlay import GridOverlay

# --- Global Variables ---
robot_ip = "192.168.59.19"
robot_port = 9999
start_node = None
target_node = None
client_socket = None
input_ready = threading.Event()
connection_failed = threading.Event()
connected = threading.Event()

# Ball detection state
detect_balls = True  # Toggle for ball detection
ball_data = {
    'white_balls': {'pixels': [], 'grid': []},
    'orange_balls': {'pixels': [], 'grid': []}
}

# --- Grid & Webcam Setup ---
grid = Grid(1800, 1200, 12)


def handle_obstacle_marked(gx, gy):
    node = grid.get_node(gx, gy)
    if node:
        grid.add_obstacle(node)


grid_overlay = GridOverlay(grid.width, grid.height, grid.density, on_mark_obstacle=handle_obstacle_marked)

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if not ret:
    print("Webcam couldn't be opened.")
    exit()

# --- OpenCV Window Setup ---
WINDOW_NAME = "Webcam Feed"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(WINDOW_NAME, grid_overlay.mouse_events)


# --- Connection Thread ---
def connect_to_robot():
    global client_socket
    print(f"Connecting to robot at {robot_ip}:{robot_port}...")
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((robot_ip, robot_port))
        print("Connected to robot!")
        connected.set()
        input_ready.set()

        # Send all obstacles in one OBSTACLE command
        print("Sending obstacles to robot...")
        obstacle_nodes = [
            node
            for col in grid.grid
            for node in col
            if node.is_obstacle
        ]

        if obstacle_nodes:
            obstacle_str = " ".join(f"{{{n.x},{n.y}}}" for n in obstacle_nodes)
            msg = f"OBSTACLE {obstacle_str}\n"
            client_socket.sendall(msg.encode())
            print(f"Sent: {msg.strip()}")
        else:
            print("No obstacles to send.")

        start_input_loop()
    except Exception as e:
        print(f"Failed to connect: {e}")
        connection_failed.set()


# --- Input Loop Thread ---
def start_input_loop():
    global start_node, target_node, client_socket

    print("Enter command like:")
    print("  MOVE {1,2} {3,2}")
    print("Type 'q' to quit.")

    while connected.is_set():
        try:
            command = input("Command: ").strip()
            if command.lower() == 'q':
                break

            if command.startswith("MOVE"):
                coords = command.split()[1:]
                if len(coords) != 2:
                    print("MOVE requires exactly 2 coordinates: START and TARGET.")
                    continue

                parsed = []
                for c in coords:
                    c = c.strip("{}")
                    try:
                        x, y = map(int, c.split(","))
                        node = grid.get_node(x, y)
                        if node is None:
                            raise ValueError()
                        parsed.append(node)
                    except:
                        print(f"Invalid coordinate: {c}")
                        break

                if len(parsed) == 2:
                    start_node = parsed[0]
                    target_node = parsed[1]
                    client_socket.sendall((command + "\n").encode())
                    print("Sent command:", command)
                else:
                    print("Could not parse MOVE command.")
            else:
                print("Unknown command.")

        except Exception as e:
            print("Error:", e)
            break

    connected.clear()
    input_ready.clear()
    try:
        client_socket.close()
    except:
        pass
    print("Disconnected from robot.")


# --- Main Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Keep a clean copy of the original frame for ball detection
    original_frame = frame.copy()

    # Ball detection on original frame (before grid overlay)
    if detect_balls:
        ball_data = find_ping_pong_balls(original_frame, grid_overlay)
        frame = draw_ball_detections(frame, ball_data)

        # Display ball coordinates
        white_txt = f"White: {len(ball_data['white_balls']['grid'])} balls"
        orange_txt = f"Orange: {len(ball_data['orange_balls']['grid'])} balls"
        cv2.putText(frame, white_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, orange_txt, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

    # Draw grid overlay after ball detection
    frame = grid_overlay.draw(frame)

    # Connection Status Display
    status_text = "Press 'C' to Connect" if not connected.is_set() else "Connected"
    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    # Ball detection status
    detection_status = "Ball Detection: ON" if detect_balls else "Ball Detection: OFF (Press 'D')"
    cv2.putText(frame, detection_status, (frame.shape[1] - 300, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

    cv2.imshow(WINDOW_NAME, frame)

    if connection_failed.is_set():
        print("Robot connection failed.")
        break

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c') and not connected.is_set():
        connection_failed.clear()
        threading.Thread(target=connect_to_robot, daemon=True).start()
    elif key == ord('d'):
        detect_balls = not detect_balls
        if detect_balls:
            print("Ball detection enabled")
        else:
            print("Ball detection disabled")

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
print("Program exited cleanly.")