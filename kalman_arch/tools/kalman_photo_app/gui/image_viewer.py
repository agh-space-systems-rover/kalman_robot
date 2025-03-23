import os
import string
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import PIL

from utils.file_colors import color_lens, get_color_emoji_cube, get_image_array
from utils.position_calculate import calculate_position
from utils.timestamp import extract_timestamp
from utils.trajectory import TrajectoryReader


class ImageViewer:
    def __init__(self, folder_path, trajectory: TrajectoryReader, out_folder):
        self.trajectory = trajectory
        self.folder_path = folder_path
        self.out_folder = out_folder
        self.sort_ascending = True
        self.sort_method = "none"

        self.color_filter = {
            "red": False,
            "green": False,
            "blue": False,
            "white": False,
        }
        self.images = get_image_array(
            self.folder_path, self.color_filter, self.sort_method, self.sort_ascending
        )

        if not self.images:
            raise ValueError("Nie znaleziono żadnych obrazów w podanym folderze.")
        self.index = 0

        self.root = tk.Tk()
        self.root.title("Kalman Image Viewer ")
        self.root.geometry("1000x600")
        # Create main layout frames
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(expand=True, fill=tk.BOTH)

        # Left frame for image display
        self.left_frame = tk.Frame(self.main_frame)
        self.left_frame.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        # Right frame for image list
        self.right_frame = tk.Frame(self.main_frame, width=400, bg="#f0f0f0")
        self.right_frame.pack(side=tk.RIGHT, fill=tk.Y)
        self.right_frame.pack_propagate(False)

        # Image display label
        self.label = tk.Label(self.left_frame)
        self.label.pack(expand=True, fill=tk.BOTH)

        # Color filter checkboxes
        self.checkbox_frame = tk.Frame(self.right_frame, bg="#f0f0f0")
        self.checkbox_frame.pack(fill=tk.X, pady=(10, 0), padx=10)

        self.filter_label = tk.Label(
            self.checkbox_frame, text="Filter by color:", bg="#f0f0f0"
        )
        self.filter_label.pack(anchor=tk.W)

        # Checkbox variables
        self.red_var = tk.BooleanVar()
        self.green_var = tk.BooleanVar()
        self.blue_var = tk.BooleanVar()
        self.white_var = tk.BooleanVar()

        # Create checkboxes
        self.red_cb = tk.Checkbutton(
            self.checkbox_frame, text="Red", variable=self.red_var, bg="#f0f0f0"
        )
        self.red_cb.pack(side=tk.LEFT, padx=5)

        self.green_cb = tk.Checkbutton(
            self.checkbox_frame, text="Green", variable=self.green_var, bg="#f0f0f0"
        )
        self.green_cb.pack(side=tk.LEFT, padx=5)

        self.blue_cb = tk.Checkbutton(
            self.checkbox_frame, text="Blue", variable=self.blue_var, bg="#f0f0f0"
        )
        self.blue_cb.pack(side=tk.LEFT, padx=5)

        self.white_cb = tk.Checkbutton(
            self.checkbox_frame, text="White", variable=self.white_var, bg="#f0f0f0"
        )
        self.white_cb.pack(side=tk.LEFT, padx=5)

        self.red_cb.config(command=self.handle_checkboxes)
        self.green_cb.config(command=self.handle_checkboxes)
        self.blue_cb.config(command=self.handle_checkboxes)
        self.white_cb.config(command=self.handle_checkboxes)

        # Sorting options - Add this new section
        self.sort_frame = tk.Frame(self.right_frame, bg="#f0f0f0")
        self.sort_frame.pack(fill=tk.X, pady=(10, 0), padx=10)

        self.sort_label = tk.Label(self.sort_frame, text="Sort by:", bg="#f0f0f0")
        self.sort_label.pack(anchor=tk.W)

        # Radio button variable
        self.sort_var = tk.StringVar(value="none")  # Default sort by none

        # Radio buttons for sorting options
        sort_options_frame = tk.Frame(self.sort_frame, bg="#f0f0f0")
        sort_options_frame.pack(fill=tk.X)

        self.color_rb = tk.Radiobutton(
            sort_options_frame,
            text="None",
            variable=self.sort_var,
            value="none",
            command=self.handle_radio_sorting,
            bg="#f0f0f0",
        )
        self.color_rb.pack(side=tk.LEFT, padx=5)

        self.accuracy_rb = tk.Radiobutton(
            sort_options_frame,
            text="Accuracy",
            variable=self.sort_var,
            value="accuracy",
            command=self.handle_radio_sorting,
            bg="#f0f0f0",
        )
        self.accuracy_rb.pack(side=tk.LEFT, padx=5)

        self.distance_rb = tk.Radiobutton(
            sort_options_frame,
            text="Distance",
            variable=self.sort_var,
            value="distance",
            command=self.handle_radio_sorting,
            bg="#f0f0f0",
        )
        self.distance_rb.pack(side=tk.LEFT, padx=5)

        # Direction toggle button
        self.direction_button = tk.Button(
            self.sort_frame,
            text="↑ Ascending",
            command=self.toggle_direction,
            bg="#e0e0e0",
            relief=tk.RAISED,
        )
        self.direction_button.pack(pady=(5, 5), anchor=tk.W)

        # Image list with scrollbar
        self.list_label = tk.Label(
            self.right_frame, text="Available Images", bg="#f0f0f0"
        )
        self.list_label.pack(pady=(10, 5))

        self.list_frame = tk.Frame(self.right_frame)
        self.list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.scrollbar = tk.Scrollbar(self.list_frame)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.image_listbox = tk.Listbox(
            self.list_frame, yscrollcommand=self.scrollbar.set
        )
        self.image_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.config(command=self.image_listbox.yview)

        # Populate the listbox with image filenames
        for img_path in self.images:
            img_path_txt = img_path.replace(".jpg", ".txt")
            self.image_listbox.insert(
                tk.END, get_color_emoji_cube(img_path_txt) + os.path.basename(img_path)
            )

        # Bind listbox selection to change image
        self.image_listbox.bind("<<ListboxSelect>>", self.on_image_select)

        # Description label
        self.cubes_descriptions = []
        self.descr = tk.Label(
            self.left_frame, text="co za wspaniałe zdjęcie", font="Iosevka 24 bold"
        )
        self.descr.pack()

        # Key bindings
        self.root.bind("<Left>", self.show_previous)
        self.root.bind("<Right>", self.show_next)

    # Handle checkbox changes
    def handle_checkboxes(self):
        self.red_var.get()
        self.green_var.get()
        self.blue_var.get()
        self.white_var.get()

        self.color_filter = {
            "red": self.red_var.get(),
            "green": self.green_var.get(),
            "blue": self.blue_var.get(),
            "white": self.white_var.get(),
        }
        self.images = get_image_array(
            self.folder_path, self.color_filter, self.sort_method, self.sort_ascending
        )

        if not self.images:
            raise ValueError("Nie znaleziono żadnych obrazów w podanym folderze.")
        self.index = 0

        # Clear the listbox
        self.image_listbox.delete(0, tk.END)

        # Repopulate the listbox with filtered image filenames
        for img_path in self.images:
            img_path_txt = img_path.replace(".jpg", ".txt")
            self.image_listbox.insert(
                tk.END, get_color_emoji_cube(img_path_txt) + os.path.basename(img_path)
            )
        self.show_image()

    # Handle radio button sorting
    def handle_radio_sorting(self):
        sort_method = self.sort_var.get()
        print(
            f"Sorting by: {sort_method}, direction: {'ascending' if self.sort_ascending else 'descending'}"
        )

        self.sort_method = sort_method

        self.images = get_image_array(
            self.folder_path, self.color_filter, self.sort_method, self.sort_ascending
        )
        self.root.title(f"Kalman Image Viewer - Sorting by {sort_method}")

        # Clear the listbox
        self.image_listbox.delete(0, tk.END)

        # Repopulate the listbox with filtered image filenames
        for img_path in self.images:
            img_path_txt = img_path.replace(".jpg", ".txt")
            self.image_listbox.insert(
                tk.END, get_color_emoji_cube(img_path_txt) + os.path.basename(img_path)
            )
        self.index = 0

        self.show_image()

    # Toggle sort direction
    def toggle_direction(self):
        self.sort_ascending = not self.sort_ascending
        if self.sort_ascending:
            self.direction_button.config(text="↑ Ascending")
        else:
            self.direction_button.config(text="↓ Descending")
        # Trigger re-sorting with new direction
        self.handle_radio_sorting()

    def show_image(self):
        image_path = self.images[self.index]
        pil_image = Image.open(image_path)
        # Dopasowanie rozmiaru obrazu do okna (opcjonalnie)
        width, height = pil_image.size
        max_size = (1600, 900)
        pil_image.thumbnail(max_size, PIL.Image.LANCZOS)
        tk_image = ImageTk.PhotoImage(pil_image)
        self.label.config(image=tk_image)
        self.label.image = tk_image

        # Update listbox selection to match the current image
        self.image_listbox.selection_clear(0, tk.END)
        self.image_listbox.selection_set(self.index)
        self.image_listbox.see(self.index)

        self.parse_imginfo(image_path)

    def on_image_select(self, event):
        # Get selected index from listbox and update the current image
        selection = self.image_listbox.curselection()
        if selection:
            self.index = selection[0]
            self.show_image()
            # Ensure the selected item is visible in the listbox
            self.image_listbox.see(self.index)

    def run(self):
        # Select the first item in the listbox
        self.image_listbox.selection_set(0)
        self.show_image()
        self.root.mainloop()

    def parse_imginfo(self, image_path):
        path_txt = image_path.replace(".jpg", ".txt")
        with open(path_txt, "r") as file:
            raw_data = file.read()

        self.cubes_descriptions = []
        num = 0
        for line in raw_data.split("\n"):
            self.cubes_descriptions.append(str(num + 1) + ": " + line)
            num += 1

        label_str = "\n".join(self.cubes_descriptions)
        self.descr.config(text=label_str)

        for i, description in enumerate(self.cubes_descriptions):
            self.root.bind(
                str(i + 1),
                lambda event, desc=description, path=image_path: self.show_popup(
                    desc, path
                ),
            )

    def show_popup(self, description, image_path):
        popup = tk.Toplevel(self.root)
        popup.title("Szczegóły")

        file_content = ""
        timestamp = extract_timestamp(image_path)
        parts = description.split()
        if len(parts) >= 5:
            try:
                x, y, z = map(float, parts[-3:])
            except ValueError:
                file_content = "Invalid description format for coordinates."
        else:
            file_content = "Description does not contain enough data for coordinates."

        print(f"Local coordinates: {x}, {y}, {z}")
        print(f"Timestamp: {timestamp}")
        cube_coords = calculate_position((x, y, z), timestamp, self.trajectory)
        global_coords = (
            f"{cube_coords["x"]:.3f} {cube_coords["y"]:.3f} {cube_coords["z"]:.3f}"
        )

        label = tk.Label(
            popup,
            text=description + f"\nGlobal coordinates: {global_coords}",
            font="Iosevka 16",
        )
        label.pack(pady=10, padx=10)
        popup.bind("<Escape>", lambda event: popup.destroy())
        popup.bind(
            "<space>",
            lambda event: (
                self.save_to_file(image_path, global_coords, description),
                popup.destroy(),
            ),
        )

    def save_to_file(self, filename, coords, description):
        print(filename)
        print(coords)
        color = description.split()[1] if len(description.split()) > 1 else "unknown"
        new_filename = (
            self.out_folder + "/" + filename.split("/")[-1].replace(".jpg", ".txt")
        )
        with open(new_filename, "w") as file:
            file.write(f"{color}: {coords}\n")
        print(f"Saved to {new_filename}")

    def show_previous(self, event=None):
        self.index = (self.index - 1) % len(self.images)
        self.show_image()

    def show_next(self, event=None):
        self.index = (self.index + 1) % len(self.images)
        self.show_image()
