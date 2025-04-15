import cv2

class GridUtil:
    def __init__(self, grid):
        self.grid = grid
        self.density = grid.density

        # These will be computed dynamically for drawing
        self.grid_width = 0
        self.grid_height = 0

        # Button setup
        self.button_areas = []
        self.create_button_layout()

    def create_button_layout(self):
        button_width = 100
        button_height = 30
        start_x = 10
        start_y = 50
        spacing = 10

        self.button_areas = [
            ("Density+", (start_x, start_y, start_x + button_width, start_y + button_height), self.increase_density),
            ("Density-", (start_x, start_y + (button_height + spacing), start_x + button_width, start_y + (button_height + spacing) + button_height), self.decrease_density),
        ]

    def draw(self, frame, window_width, window_height, path=None):
        # Styling
        text_color = (255, 255, 255)
        text_outline = (0, 0, 0)
        button_bg_color = (150, 150, 150)
        button_border_color = (255, 255, 255)

        # Get aspect ratio of real-world grid
        real_aspect = self.grid.width / self.grid.height
        draw_area_width = int(window_width * 0.8)
        draw_area_height = int(window_height * 0.8)

        # Fit the real-world aspect ratio inside the draw area
        if draw_area_width / draw_area_height > real_aspect:
            self.grid_height = draw_area_height
            self.grid_width = int(draw_area_height * real_aspect)
        else:
            self.grid_width = draw_area_width
            self.grid_height = int(draw_area_width / real_aspect)

        # Calculate cell sizes
        cell_width = self.grid_width // self.density
        cell_height = self.grid_height // self.density

        # Adjust width/height to fit whole cells
        self.grid_width = cell_width * self.density
        self.grid_height = cell_height * self.density

        # Center the grid
        start_x = (window_width - self.grid_width) // 2
        start_y = (window_height - self.grid_height) // 2

        # Draw grid
        for x in range(self.density):
            for y in range(self.density):
                x1 = start_x + x * cell_width
                y1 = start_y + y * cell_height
                x2 = x1 + cell_width
                y2 = y1 + cell_height

                node = self.grid.get_node(x, y)

                # Fill obstacle cells
                if hasattr(node, "is_obstacle") and node.is_obstacle:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 150), -1)  # Dark red fill

                # Draw cell border
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

                # Draw coordinate text
                coord_text = f"{x},{y}"
                cv2.putText(frame, coord_text, (x1 + 3, y1 + 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, text_outline, 2, cv2.LINE_AA)
                cv2.putText(frame, coord_text, (x1 + 3, y1 + 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, text_color, 1, cv2.LINE_AA)

        # Draw path
        if path:
            for i, node in enumerate(path):
                cx = start_x + node.x * cell_width + cell_width // 2
                cy = start_y + node.y * cell_height + cell_height // 2

                # Start node = green, end = red, middle = blue
                if i == 0:
                    color = (0, 255, 0)
                elif i == len(path) - 1:
                    color = (0, 0, 255)
                else:
                    color = (255, 0, 0)

                radius = min(cell_width, cell_height) // 3
                cv2.circle(frame, (cx, cy), radius, color, -1)

        # Info text
        info_text = f'Field: {self.grid.width}x{self.grid.height} cm | Density: {self.density}x{self.density}'
        cv2.putText(frame, info_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, text_outline, 2, cv2.LINE_AA)
        cv2.putText(frame, info_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, text_color, 1, cv2.LINE_AA)

        # Draw buttons
        for label, (x1, y1, x2, y2), _ in self.button_areas:
            cv2.rectangle(frame, (x1, y1), (x2, y2), button_bg_color, -1)
            cv2.rectangle(frame, (x1, y1), (x2, y2), button_border_color, 1)

            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            text_x = x1 + ((x2 - x1) - label_size[0]) // 2
            text_y = y1 + ((y2 - y1) + label_size[1]) // 2
            cv2.putText(frame, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, text_outline, 2, cv2.LINE_AA)
            cv2.putText(frame, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, text_color, 1, cv2.LINE_AA)

        return frame

    def handle_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for label, (x1, y1, x2, y2), action in self.button_areas:
                if x1 <= x <= x2 and y1 <= y <= y2:
                    action()

    def increase_density(self):
        self.density += 1
        self.apply_changes()

    def decrease_density(self):
        if self.density > 1:
            self.density -= 1
            self.apply_changes()

    def apply_changes(self):
        self.grid.density = self.density
        self.grid.create_grid()
