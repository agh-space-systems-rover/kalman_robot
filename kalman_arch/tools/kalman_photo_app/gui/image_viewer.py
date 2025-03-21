import os
import string
import tkinter as tk
from PIL import Image, ImageTk
import PIL

from utils.position_calculate import calculate_position
from utils.timestamp import extract_timestamp
from utils.trajectory import TrajectoryReader


class ImageViewer:
    def __init__(self, folder_path, trajectory: TrajectoryReader, out_folder):
        self.trajectory = trajectory
        self.out_folder = out_folder
        supported_extensions = (".jpg", ".jpeg", ".png", ".gif")
        self.images = [
            os.path.join(folder_path, f)
            for f in os.listdir(folder_path)
            if f.lower().endswith(supported_extensions)
        ]
        if not self.images:
            raise ValueError("Nie znaleziono żadnych obrazów w podanym folderze.")
        self.index = 0

        self.root = tk.Tk()
        self.root.minsize(800, 600)
        self.root.title("Kalman Image Viewer ")
        self.label = tk.Label(self.root)
        self.label.pack(expand=True, fill=tk.BOTH)
        self.cubes_descriptions = []

        self.descr = tk.Label(
            self.root, text="co za wspaniałe zdjęcie", font="Iosevka 24 bold"
        )
        self.descr.pack()
        # Obsługa przycisków strzałek
        self.root.bind("<Left>", self.show_previous)
        self.root.bind("<Right>", self.show_next)

    def run(self):
        self.show_image()
        self.root.mainloop()

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

        self.parse_imginfo(image_path)

    def parse_imginfo(self, image_path):
        path_txt = image_path.replace(".jpg", ".txt")
        with open(path_txt, "r") as file:
            raw_data = file.read()

        self.cubes_descriptions = []
        num = 0
        for line in raw_data.split("\n"):
            self.cubes_descriptions.append(str(num + 1) + ": " + line)
            num += 1

        label_str = "Filename: " + path_txt + "\n" + "\n".join(self.cubes_descriptions)
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
        base_filename = os.path.basename(filename).replace(".jpg", ".txt")
        new_filename = os.path.join(self.out_folder, base_filename)
        with open(new_filename, "w") as file:
            file.write(f"{color}: {coords}\n")
        print(f"Saved to {new_filename}")

    def show_previous(self, event=None):
        self.index = (self.index - 1) % len(self.images)
        self.show_image()

    def show_next(self, event=None):
        self.index = (self.index + 1) % len(self.images)
        self.show_image()
