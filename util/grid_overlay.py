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

        # Draw only obstacles with filled red background
        for gx, gy in self.obstacles:
            cell = np.float32([
                [gx / self.grid_cols, gy / self.grid_rows],
                [(gx + 1) / self.grid_cols, gy / self.grid_rows],
                [(gx + 1) / self.grid_cols, (gy + 1) / self.grid_rows],
                [gx / self.grid_cols, (gy + 1) / self.grid_rows]
            ]) * 100

            warped = cv2.perspectiveTransform(cell.reshape(-1, 1, 2), self.matrix).reshape(-1, 2).astype(int)
            cv2.fillPoly(frame, [warped], (0, 0, 255))

        # Draw coordinate labels
        for gx in range(self.grid_cols):
            for gy in range(self.grid_rows):
                cell = np.float32([
                    [gx / self.grid_cols, gy / self.grid_rows],
                    [(gx + 1) / self.grid_cols, gy / self.grid_rows],
                    [(gx + 1) / self.grid_cols, (gy + 1) / self.grid_rows],
                    [gx / self.grid_cols, (gy + 1) / self.grid_rows]
                ]) * 100
                warped = cv2.perspectiveTransform(cell.reshape(-1, 1, 2), self.matrix).reshape(-1, 2).astype(int)
                center = np.mean(warped, axis=0).astype(int)
                label = f"({gx},{gy})"
                cv2.putText(frame, label, tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

        # Draw vertical lines
        for i in range(1, self.grid_cols):
            alpha = i / self.grid_cols
            p1 = (1 - alpha) * src[0] + alpha * src[1]
            p2 = (1 - alpha) * src[3] + alpha * src[2]
            warped = cv2.perspectiveTransform(np.float32([p1, p2]).reshape(-1, 1, 2) * 100, self.matrix)
            cv2.line(frame, tuple(warped[0][0].astype(int)), tuple(warped[1][0].astype(int)), (0, 255, 0), 1)

        # Draw horizontal lines
        for j in range(1, self.grid_rows):
            beta = j / self.grid_rows
            p1 = (1 - beta) * src[0] + beta * src[3]
            p2 = (1 - beta) * src[1] + beta * src[2]
            warped = cv2.perspectiveTransform(np.float32([p1, p2]).reshape(-1, 1, 2) * 100, self.matrix)
            cv2.line(frame, tuple(warped[0][0].astype(int)), tuple(warped[1][0].astype(int)), (0, 255, 0), 1)

        # Draw outer polygon and handles
        cv2.polylines(frame, [np.array(self.corners, np.int32).reshape((-1, 1, 2))], isClosed=True, color=(0, 255, 0), thickness=2)
        for px, py in self.corners:
            cv2.rectangle(frame, (px - self.handle_size, py - self.handle_size), (px + self.handle_size, py + self.handle_size), (0, 0, 255), -1)
        return frame

    def get_coordinate_from_pixel(self, mx, my):
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
            self.dragging_point = self.get_point_index(mx, my)

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
