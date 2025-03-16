import cv2

class GridUtil:
    def __init__(self, grid):
        self.grid = grid

        # Local copies of grid values
        self.grid_width = grid.width
        self.grid_height = grid.height
        self.density = grid.density

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
            ("Size+", (start_x, start_y, start_x + button_width, start_y + button_height), self.increase_grid_size),
            ("Size-", (start_x, start_y + (button_height + spacing), start_x + button_width, start_y + (button_height + spacing) + button_height), self.decrease_grid_size),
            ("Density+", (start_x, start_y + 2 * (button_height + spacing), start_x + button_width, start_y + 2 * (button_height + spacing) + button_height), self.increase_density),
            ("Density-", (start_x, start_y + 3 * (button_height + spacing), start_x + button_width, start_y + 3 * (button_height + spacing) + button_height), self.decrease_density),
        ]

    def draw(self, frame, window_width, window_height):
        # Styling
        text_color = (255, 255, 255)
        text_outline = (0, 0, 0)
        button_bg_color = (150, 150, 150)
        button_border_color = (255, 255, 255)

        # Draw grid centered
        start_x = (window_width - self.grid_width) // 2
        start_y = (window_height - self.grid_height) // 2

        cell_width = self.grid_width // self.density
        cell_height = self.grid_height // self.density

        for x in range(self.density):
            for y in range(self.density):
                x1 = start_x + x * cell_width
                y1 = start_y + y * cell_height
                x2 = x1 + cell_width
                y2 = y1 + cell_height

                # Draw grid cell
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

                # Draw coordinate text
                coord_text = f"{x},{y}"
                cv2.putText(frame, coord_text, (x1 + 3, y1 + 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, text_outline, 2, cv2.LINE_AA)
                cv2.putText(frame, coord_text, (x1 + 3, y1 + 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, text_color, 1, cv2.LINE_AA)

        # Info text
        info_text = f'Size: {self.grid_width}x{self.grid_height} | Density: {self.density}x{self.density}'
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

    # === Size Control ===
    def increase_grid_size(self):
        self.grid_width += 20
        self.grid_height += 20
        self.apply_changes()

    def decrease_grid_size(self):
        self.grid_width = max(100, self.grid_width - 20)
        self.grid_height = max(100, self.grid_height - 20)
        self.apply_changes()

    # === Density Control ===
    def increase_density(self):
        self.density += 1
        self.apply_changes()

    def decrease_density(self):
        if self.density > 1:
            self.density -= 1
            self.apply_changes()

    # === Apply Changes to Grid ===
    def apply_changes(self):
        self.grid.width = self.grid_width
        self.grid.height = self.grid_height
        self.grid.density = self.density
        self.grid.recreate_grid()
