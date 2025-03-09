import tkinter as tk
from tkinter import filedialog, ttk
from PIL import Image, ImageTk
from PIL.Image import Resampling
import yaml
import os

class MapAnnotator:
    def __init__(self, root):
        self.root = root
        self.root.title("Map Annotator")

        # -----------------------
        # Variables and Defaults
        # -----------------------
        self.image_path = None
        self.original_image = None  # PIL Image
        self.tk_image = None        # ImageTk for Canvas

        self.img_width = 1
        self.img_height = 1

        # Scale factors
        self.fit_scale_factor = 1.0   # fits image to canvas
        self.user_zoom_factor = 1.0   # changed by mouse wheel
        self.pan_x = 0
        self.pan_y = 0

        # Annotation data
        self.origin = None            # (pixel_x, pixel_y)
        self.tasks = []               # list of dicts: {id, type, pixel_coords}
        self.goal_id_counter = 1
        self.goal_type = tk.StringVar(value="goal")
        self.area_size_var = tk.DoubleVar(value=10.0)

        # Create GUI layout
        self._create_layout()

    def _create_layout(self):
        # Main frame -> Left: Canvas, Right: Sidebar
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # Canvas for the map
        self.canvas = tk.Canvas(self.main_frame, bg="grey")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Important: bind to <Configure> so we can detect when the canvas is resized
        self.canvas.bind("<Configure>", self.on_canvas_resize)

        # Bind mouse events for adding/removing goals, zooming
        self.canvas.bind("<Button-1>", self.on_left_click)
        self.canvas.bind("<Button-3>", self.on_right_click)
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)
        self.canvas.bind("<Motion>", self.on_mouse_move)

        # Sidebar frame
        self.sidebar = tk.Frame(self.main_frame, width=200)
        self.sidebar.pack(side=tk.RIGHT, fill=tk.Y)

        # Buttons and controls in sidebar
        load_button = tk.Button(self.sidebar, text="Load Image", command=self.load_image)
        load_button.pack(pady=5, fill=tk.X)

        tk.Label(self.sidebar, text="Area Size (image width) [m]:").pack(pady=2)
        tk.Entry(self.sidebar, textvariable=self.area_size_var).pack(pady=2, fill=tk.X)

        origin_button = tk.Button(self.sidebar, text="Set Origin",
                                  command=lambda: self.goal_type.set("origin"))
        origin_button.pack(pady=5, fill=tk.X)

        tk.Label(self.sidebar, text="Select Goal Type:").pack(pady=2)
        for gtype in ["goal", "panorama", "loop", "explore"]:
            rb = tk.Radiobutton(self.sidebar, text=gtype.capitalize(),
                                variable=self.goal_type, value=gtype)
            rb.pack(anchor=tk.W)

        save_button = tk.Button(self.sidebar, text="Save YAML", command=self.save_yaml)
        save_button.pack(pady=5, fill=tk.X)

        self.cursor_label = tk.Label(self.sidebar, text="Cursor: (x=0.0, y=0.0)")
        self.cursor_label.pack(pady=10)

    # -------------------------
    # Image Loading
    # -------------------------
    def load_image(self):
        path = filedialog.askopenfilename(
            filetypes=[("Image Files", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff")]
        )
        if not path:
            return

        self.image_path = path
        self.original_image = Image.open(path)
        self.img_width, self.img_height = self.original_image.size

        # Reset user zoom & pan
        self.user_zoom_factor = 1.0
        self.pan_x = 0
        self.pan_y = 0

        # Calculate fit scale based on the current canvas size
        self._calc_fit_scale_factor()
        self._update_display_image()
        self._draw_image()

    def _calc_fit_scale_factor(self):
        """Set fit_scale_factor so the image fits fully into the canvas."""
        c_w = self.canvas.winfo_width()
        c_h = self.canvas.winfo_height()
        if c_w < 1: c_w = 1
        if c_h < 1: c_h = 1

        scale_w = c_w / self.img_width
        scale_h = c_h / self.img_height

        # If you prefer the image to be allowed to grow if it's small, remove the '1.0' or tweak logic
        self.fit_scale_factor = min(scale_w, scale_h, 1.0)

    def _update_display_image(self):
        """Update the self.tk_image based on fit_scale_factor * user_zoom_factor."""
        if not self.original_image:
            return
        effective_scale = self.fit_scale_factor * self.user_zoom_factor
        disp_w = int(self.img_width * effective_scale)
        disp_h = int(self.img_height * effective_scale)

        # If for some reason they become <1, clamp to 1
        disp_w = max(disp_w, 1)
        disp_h = max(disp_h, 1)

        # Resize with LANCZOS or your preferred filter
        resized = self.original_image.resize((disp_w, disp_h), Resampling.LANCZOS)
        self.tk_image = ImageTk.PhotoImage(resized)

    def _draw_image(self):
        self.canvas.delete("all")
        if self.tk_image is not None:
            self.canvas.create_image(
                self.pan_x, self.pan_y,
                image=self.tk_image,
                anchor=tk.NW
            )
        self._draw_origin()
        self._draw_goals()

    # -------------------------
    # Drawing Annotations
    # -------------------------
    def _draw_origin(self):
        """Draw the origin marker if it exists."""
        if self.origin is None:
            return
        ox, oy = self.origin
        cx, cy = self.to_canvas_coords(ox, oy)
        r = 5
        self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill="red", outline="black", width=2)
        self.canvas.create_text(cx, cy-15, text="Origin", fill="red")

    def _draw_goals(self):
        """Draw the annotated goals on the canvas."""
        for task in self.tasks:
            x, y = task["pixel_coords"]
            cx, cy = self.to_canvas_coords(x, y)

            r = 5
            color = "blue"
            if task["type"] == "panorama":
                color = "green"
            elif task["type"] == "loop":
                color = "orange"
            elif task["type"] == "explore":
                color = "purple"

            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=color, outline="black", width=1)
            text_label = f"{task['id']} ({task['type']})"
            self.canvas.create_text(cx, cy-15, text=text_label, fill=color)

    # -------------------------
    # Mouse/Zoom Events
    # -------------------------
    def on_canvas_resize(self, event):
        """
        Called whenever the canvas is resized. Recalculate fit scale factor,
        then update and redraw the image while preserving user_zoom_factor.
        """
        if not self.original_image:
            return
        # Re-calc how to fit the image to the new canvas size
        self._calc_fit_scale_factor()
        self._update_display_image()
        self._draw_image()

    def on_left_click(self, event):
        """Left-click: either set origin or place a goal."""
        # Convert from canvas to actual image pixel coords
        x, y = self.to_image_coords(event.x, event.y)
        if x < 0 or y < 0 or x > self.img_width or y > self.img_height:
            return  # ignore clicks outside the image

        current_mode = self.goal_type.get()
        if current_mode == "origin":
            self.origin = (x, y)
        else:
            new_task = {
                "id": self.goal_id_counter,
                "type": current_mode,
                "pixel_coords": (x, y)
            }
            self.tasks.append(new_task)
            self.goal_id_counter += 1

        self._draw_image()

    def on_right_click(self, event):
        """Right-click: remove the closest goal if we click near it."""
        x, y = self.to_image_coords(event.x, event.y)
        threshold = 10  # pixel threshold in image coords
        closest_task = None
        closest_dist = float("inf")

        for task in self.tasks:
            tx, ty = task["pixel_coords"]
            dist = ((tx - x)**2 + (ty - y)**2)**0.5
            if dist < closest_dist:
                closest_dist = dist
                closest_task = task

        if closest_task and closest_dist <= threshold:
            self.tasks.remove(closest_task)
            self._draw_image()

    def on_mouse_wheel(self, event):
        """Zoom in/out depending on scroll direction."""
        # Update the user_zoom_factor
        if event.delta > 0:   # scroll up
            self.user_zoom_factor *= 1.1
        else:                 # scroll down
            self.user_zoom_factor *= 0.9

        # Bound it so it doesn't go nuts
        self.user_zoom_factor = max(0.1, min(self.user_zoom_factor, 10.0))

        # Update the displayed image
        self._update_display_image()
        self._draw_image()

    def on_mouse_move(self, event):
        """Update cursor position in map coordinates if inside the image area."""
        x_img, y_img = self.to_image_coords(event.x, event.y)
        if (0 <= x_img <= self.img_width) and (0 <= y_img <= self.img_height):
            x_map, y_map = self.image_to_map_coords(x_img, y_img)
            self.cursor_label.config(
                text=f"Cursor: (x={x_map:.2f}, y={y_map:.2f})"
            )
        else:
            self.cursor_label.config(text="Cursor: (x=---, y=---)")

    # -------------------------
    # Coordinate Conversions
    # -------------------------
    def to_canvas_coords(self, img_x, img_y):
        """Convert image pixel coords -> canvas coords."""
        # Effective scale is the product of fit_scale_factor and user_zoom_factor
        effective_scale = self.fit_scale_factor * self.user_zoom_factor
        cx = img_x * effective_scale + self.pan_x
        cy = img_y * effective_scale + self.pan_y
        return cx, cy

    def to_image_coords(self, can_x, can_y):
        """Convert canvas coords -> image pixel coords."""
        effective_scale = self.fit_scale_factor * self.user_zoom_factor
        if effective_scale == 0:
            return 0, 0
        x = (can_x - self.pan_x) / effective_scale
        y = (can_y - self.pan_y) / effective_scale
        return x, y

    def image_to_map_coords(self, x_img, y_img):
        """
        Convert from image pixel coords to 'map' coords in meters,
        using area_size and the chosen origin.
        """
        area_size = self.area_size_var.get()
        if self.img_width == 0:
            return 0, 0

        # Meters per pixel (assuming area_size is width in meters)
        m_per_px = area_size / float(self.img_width)

        # Shift relative to origin, invert y if desired
        if self.origin:
            ox, oy = self.origin
            dx = (x_img - ox) * m_per_px
            dy = (y_img - oy) * m_per_px
            return -dy, -dx
        else:
            return x_img * m_per_px, y_img * m_per_px

    # -------------------------
    # Save YAML
    # -------------------------
    def save_yaml(self):
        if not self.image_path:
            return

        filename = os.path.basename(self.image_path)
        area_size = self.area_size_var.get()

        if self.origin is not None:
            origin_map = self.image_to_map_coords(*self.origin)
        else:
            origin_map = (0.0, 0.0)

        tasks_list = []
        for task in self.tasks:
            t_id = task["id"]
            t_type = task["type"]
            map_coords = self.image_to_map_coords(*task["pixel_coords"])
            tasks_list.append({
                "id": t_id,
                "type": t_type,
                "coordinates": [round(map_coords[0], 3), round(map_coords[1], 3)]
            })

        data = {
            "filename": filename,
            "area_size": area_size,
            "origin": [round(origin_map[0], 3), round(origin_map[1], 3)],
            "tasks": tasks_list
        }

        save_path = filedialog.asksaveasfilename(
            defaultextension=".yaml",
            filetypes=[("YAML Files", "*.yaml"), ("All Files", "*.*")]
        )
        if save_path:
            with open(save_path, "w") as f:
                yaml.dump(data, f, sort_keys=False)
            print(f"Saved YAML to {save_path}")


if __name__ == "__main__":
    root = tk.Tk()
    # Give it an initial default size
    root.geometry("900x600")
    app = MapAnnotator(root)
    root.mainloop()
