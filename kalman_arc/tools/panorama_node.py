import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import messagebox
import threading
import queue
import math
import io
import os
import glob
import cv2
import datetime
from pathlib import Path

class PanoramaNode(Node):
    def __init__(self, data_queue):
        super().__init__('panorama_collector_node')
        self.data_queue = data_queue
        self.camera_subscriber = self.create_subscription(
            CompressedImage, '/d455_front/color/image_raw/compressed', self.camera_callback, 5)
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 5)
        self.current_yaw = 0.0

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = (math.degrees(yaw_rad) + 360) % 360

    def camera_callback(self, msg):
        try:
            self.data_queue.put_nowait((bytes(msg.data), self.current_yaw))
        except queue.Full:
            pass

class PanoramaApp:
    def __init__(self, root, data_queue):
        self.root = root
        self.data_queue = data_queue
        self.root.title("Panorama Creator")
        self.root.geometry("800x700")
        
        self.is_recording = False
        self.saved_count = 0
        self.angle_threshold = 45.0
        self.last_saved_yaw = -999.0
        self.accumulated_angle = 0.0
        self.prev_yaw = None
        
        # Directory setup
        script_dir = Path(__file__).resolve()
        self.base_dir = "panorama_data"
        self.base_dir = os.path.join(script_dir.parent, self.base_dir)
        self.panoramas_dir = os.path.join(self.base_dir, "final_panoramas")
        self.current_frames_dir = None
        os.makedirs(self.panoramas_dir, exist_ok=True)

        self.setup_ui()
        self.update_loop()

    def setup_ui(self):
        self.image_label = tk.Label(self.root, text="Waiting for camera...")
        self.image_label.pack(pady=10)
        
        self.status_label = tk.Label(self.root, text="Status: Ready", font=("Arial", 14, "bold"), fg="blue")
        self.status_label.pack(pady=5)
        
        self.info_label = tk.Label(self.root, text="Yaw: 0° | Progress: 0/360°", font=("Arial", 12))
        self.info_label.pack(pady=5)

        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=15)
        
        tk.Button(btn_frame, text="▶ Start", command=self.start_recording, bg="lightgreen", width=10).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="↺ Reset", command=self.reset_recording, bg="gold", width=10).pack(side=tk.LEFT, padx=5)

    def reset_recording(self):
        self.is_recording = False
        self.accumulated_angle = 0.0
        self.saved_count = 0
        self.prev_yaw = None
        self.last_saved_yaw = -999.0
            
        self.status_label.configure(text="Status: Ready", fg="blue")
        self.info_label.configure(text="Yaw: 0° | Progress: 0/360°")
        print("Sesja zresetowana.")
    def update_loop(self):
        while not self.data_queue.empty():
            raw_bytes, yaw = self.data_queue.get_nowait()
            pil_img = Image.open(io.BytesIO(raw_bytes)).convert('RGB')
            pil_img.thumbnail((640, 480))
            tk_img = ImageTk.PhotoImage(image=pil_img)
            self.image_label.configure(image=tk_img)
            self.image_label.image = tk_img

            if self.is_recording:
                self.process_recording(yaw, raw_bytes)
        self.root.after(30, self.update_loop)

    def process_recording(self, yaw, raw_bytes):
        if self.prev_yaw is not None:
            diff = (yaw - self.prev_yaw + 180) % 360 - 180
            self.accumulated_angle += abs(diff)
        self.prev_yaw = yaw

        if abs(yaw - self.last_saved_yaw) % 360 >= self.angle_threshold:
            filename = os.path.join(self.current_frames_dir, f"frame_{self.saved_count:03d}.jpg")
            with open(filename, 'wb') as f: f.write(raw_bytes)
            self.last_saved_yaw = yaw
            self.saved_count += 1
            self.info_label.configure(text=f"Yaw: {yaw:.1f}° | Progress: {self.accumulated_angle:.0f}/360° | Saved: {self.saved_count}")

        if self.accumulated_angle >= 350:
            self.finish_panorama()

    def start_recording(self):
        self.is_recording = True
        self.accumulated_angle = 0.0
        self.saved_count = 0
        self.current_frames_dir = os.path.join(self.base_dir, f"session_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(self.current_frames_dir, exist_ok=True)
        self.status_label.configure(text="Status: Recording...", fg="red")

    def finish_panorama(self):
        self.is_recording = False
        self.status_label.configure(text="Status: Stitching...", fg="orange")
        threading.Thread(target=self.process_stitching, daemon=True).start()

    def process_stitching(self):
        image_files = sorted(glob.glob(os.path.join(self.current_frames_dir, "*.jpg")))
        images = [cv2.imread(f) for f in image_files if cv2.imread(f) is not None]
        
        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        status, stitched = stitcher.stitch(images)
        
        if status == cv2.Stitcher_OK:
            path = os.path.join(self.panoramas_dir, f"panorama_{datetime.datetime.now().strftime('%Y%m%d_%H%M')}.jpg")
            cv2.imwrite(path, stitched)
            self.root.after(0, lambda: messagebox.showinfo("Success", f"Saved: {path}"))
        else:
            self.root.after(0, lambda: messagebox.showerror("Error", f"Stitching failed with code: {status}"))
        
        self.root.after(0, self.reset_recording)

def main():
    rclpy.init()
    data_queue = queue.Queue(maxsize=2)
    node = PanoramaNode(data_queue)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    root = tk.Tk()
    app = PanoramaApp(root, data_queue)
    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()