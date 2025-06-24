import cv2
import numpy as np

class GridOverlay:
    def __init__(self, width=1800, height=1200, density=4, on_mark_obstacle=None):
        self.width = width
        self.height = height
        self.grid_rows = self.grid_cols = density
        self.handle_size = 10
        self.corners = [(100, 100), (300, 100), (300, 300), (100, 300)]
        self.obstacles = []
        self.dragging_point = -1
        self.matrix = None
        self.on_mark_obstacle = on_mark_obstacle

    def draw(self, frame):
        src = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
        dst = np.float32(self.corners[:4])
        self.matrix = cv2.getPerspectiveTransform(src * 100, dst)

        # Draw red obstacle tiles
        for gx, gy in self.obstacles:
            cell = np.float32([
                [gx / self.grid_cols, gy / self.grid_rows],
                [(gx + 1) / self.grid_cols, gy / self.grid_rows],
                [(gx + 1) / self.grid_cols, (gy + 1) / self.grid_rows],
                [gx / self.grid_cols, (gy + 1) / self.grid_rows]
            ]) * 100

            warped = cv2.perspectiveTransform(cell.reshape(-1, 1, 2), self.matrix).reshape(-1, 2).astype(int)
            cv2.fillPoly(frame, [warped], (0, 0, 255))

        # Draw X coordinate labels (top)
        for gx in range(self.grid_cols):
            label = str(gx)
            alpha = (gx + 0.5) / self.grid_cols
            top = (1 - alpha) * src[0] + alpha * src[1]
            warped_top = cv2.perspectiveTransform(np.float32([top]).reshape(-1, 1, 2) * 100, self.matrix)[0][0]
            cv2.putText(frame, label, tuple(warped_top.astype(int) - [0, 10]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Draw Y coordinate labels (left)
        for gy in range(self.grid_rows):
            label = str(gy)
            beta = (gy + 0.5) / self.grid_rows
            left = (1 - beta) * src[0] + beta * src[3]
            warped_left = cv2.perspectiveTransform(np.float32([left]).reshape(-1, 1, 2) * 100, self.matrix)[0][0]
            cv2.putText(frame, label, tuple(warped_left.astype(int) - [25, 0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Draw grid lines
        for i in range(1, self.grid_cols):
            alpha = i / self.grid_cols
            p1 = (1 - alpha) * src[0] + alpha * src[1]
            p2 = (1 - alpha) * src[3] + alpha * src[2]
            warped = cv2.perspectiveTransform(np.float32([p1, p2]).reshape(-1, 1, 2) * 100, self.matrix)
            cv2.line(frame, tuple(warped[0][0].astype(int)), tuple(warped[1][0].astype(int)), (0, 255, 0), 1)

        for j in range(1, self.grid_rows):
            beta = j / self.grid_rows
            p1 = (1 - beta) * src[0] + beta * src[3]
            p2 = (1 - beta) * src[1] + beta * src[2]
            warped = cv2.perspectiveTransform(np.float32([p1, p2]).reshape(-1, 1, 2) * 100, self.matrix)
            cv2.line(frame, tuple(warped[0][0].astype(int)), tuple(warped[1][0].astype(int)), (0, 255, 0), 1)

        # Draw outer polygon
        cv2.polylines(frame, [np.array(self.corners, np.int32).reshape((-1, 1, 2))], isClosed=True, color=(0, 255, 0), thickness=2)

        # Draw corner handles
        for i, (px, py) in enumerate(self.corners):
            radius = 6
            if i == self.dragging_point:
                overlay = frame.copy()
                cv2.circle(overlay, (px, py), radius, (0, 0, 255), -1)
                alpha = 0.08
                cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
            else:
                cv2.circle(frame, (px, py), radius, (0, 0, 255), -1)

        return frame

    def get_coordinate_from_pixel(self, mx, my, is_fhd: bool = False):
        """
        Converts a pixel coordinate (mx, my) from the image frame into a grid cell coordinate (gx, gy).

        This method uses the perspective transformation matrix to map the pixel location
        to the corresponding grid cell in the overlay. If the pixel is outside the grid bounds,
        it returns (-1, -1).

        Args:
            mx (int): The x-coordinate of the pixel in the image frame.
            my (int): The y-coordinate of the pixel in the image frame.

        Returns:
            tuple: A tuple (cell_x, cell_y) representing the grid cell coordinates.
                   Returns (-1, -1) if the pixel is outside the grid bounds or the grid cell is invalid.
        """
        # Correct for perspective by estimation
        scale = 0.9  # arbitrary scale factor, correct as needed
        cam_x = 1920/2 if is_fhd else 1280/2  # assuming 1080p/720p based on is_fhd
        cam_y = 1080/2 if is_fhd else 720/2
    
        diff_x, diff_y = mx - cam_x, my - cam_y
        mx, my = int(cam_x + diff_x * scale), int(cam_y + diff_y * scale)
        
        # Convert to node
        if self.matrix is None:
            return -1, -1
        inv_matrix = np.linalg.inv(self.matrix)
        pixel_point = np.array([[mx, my]], dtype=np.float32).reshape(-1, 1, 2)
        grid_point = cv2.perspectiveTransform(pixel_point, inv_matrix)[0][0]
        gx, gy = grid_point
        if gx < 0 or gx > 100 or gy < 0 or gy > 100:
            return -1, -1
        cell_x = int(gx * self.grid_cols / 100)
        cell_y = int(gy * self.grid_rows / 100)
        return (cell_x, cell_y) if 0 <= cell_x < self.grid_cols and 0 <= cell_y < self.grid_rows else (-1, -1)

    def get_point_index(self, mx, my):
        for i, (px, py) in enumerate(self.corners):
            if px - self.handle_size <= mx <= px + self.handle_size and py - self.handle_size <= my <= py + self.handle_size:
                return i
        return -1

    def mouse_events(self, event, mx, my, flags, param=None):
        if event == cv2.EVENT_LBUTTONDOWN:
            index = self.get_point_index(mx, my)
            if index != -1:
                self.dragging_point = index
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.get_point_index(mx, my) == -1:
                gx, gy = self.get_coordinate_from_pixel(mx, my)
                if (gx, gy) != (-1, -1):
                    if (gx, gy) in self.obstacles:
                        self.obstacles.remove((gx, gy))
                    else:
                        self.obstacles.append((gx, gy))
                        if self.on_mark_obstacle:
                            self.on_mark_obstacle(gx, gy)

        elif event == cv2.EVENT_MOUSEMOVE and self.dragging_point != -1:
            self.corners[self.dragging_point] = (mx, my)

        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging_point = -1
