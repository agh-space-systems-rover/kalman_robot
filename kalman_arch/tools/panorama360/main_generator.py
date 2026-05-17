from datetime import datetime

import cv2
import numpy as np
from pynput import keyboard


class KeyboardController:
    """Controller for keyboard input."""

    def __init__(self):
        self.current_action = None
        self.listener = keyboard.Listener(on_press=self._on_press)

    def start(self):
        self.listener.start()

    def stop(self):
        self.listener.stop()

    def _on_press(self, key):
        try:
            char = key.char.lower()
            if char in ["s", "r", "q"]:
                self.current_action = char
        except AttributeError:
            pass

    def get_action(self):
        action = self.current_action
        self.current_action = None
        return action


class CylindricalStitcher:
    """Class for stitching cylindrical frames into a panorama."""

    def __init__(self, focal_length=700):
        self.focal_length = focal_length
        self.finder = cv2.SIFT_create()
        self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        self.panorama = None
        self.added_frames_count = 0

    def reset(self):
        self.panorama = None
        self.added_frames_count = 0

    def rotate_frame(self, img, angle):
        if angle == 90:
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif angle == 180:
            return cv2.rotate(img, cv2.ROTATE_180)
        elif angle == 270:
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return img

    def warp_cylinder(self, img):
        h, w = img.shape[:2]
        y, x = np.indices((h, w))
        cx, cy = w / 2, h / 2

        theta = (x - cx) / self.focal_length
        h_ = (y - cy) / self.focal_length

        X = np.sin(theta)
        Y = h_
        Z = np.cos(theta)

        x_cyl = self.focal_length * X / Z + cx
        y_cyl = self.focal_length * Y / Z + cy

        mask = (x_cyl >= 0) & (x_cyl < w) & (y_cyl >= 0) & (y_cyl < h)
        cylindrical_img = cv2.remap(
            img, x_cyl.astype(np.float32), y_cyl.astype(np.float32), cv2.INTER_LINEAR
        )

        return cylindrical_img, (mask * 255).astype(np.uint8)

    def try_stitch(self, frame_cyl, frame_mask):
        if self.panorama is None:
            self.panorama = frame_cyl.copy()
            self.added_frames_count = 1
            return True

        kp1, des1 = self.finder.detectAndCompute(self.panorama, None)
        kp2, des2 = self.finder.detectAndCompute(frame_cyl, None)

        if des1 is None or des2 is None:
            return False

        matches = self.bf.knnMatch(des2, des1, k=2)
        good_matches = []
        for match in matches:
            if len(match) == 2:
                m, n = match
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
        if len(good_matches) <= 10:
            print("Skipped frame - too few common points.")
            return False

        src_pts = np.float32([kp2[m.queryIdx].pt for m in good_matches]).reshape(
            -1, 1, 2
        )
        dst_pts = np.float32([kp1[m.trainIdx].pt for m in good_matches]).reshape(
            -1, 1, 2
        )

        M, _ = cv2.estimateAffinePartial2D(
            src_pts, dst_pts, method=cv2.RANSAC, ransacReprojThreshold=4.0
        )
        if M is None:
            return False

        tx, ty = M[0, 2], M[1, 2]
        h_pan, w_pan = self.panorama.shape[:2]
        h_f, w_f = frame_cyl.shape[:2]

        new_w = max(w_pan, int(max(0, tx) + w_f))
        new_h = max(h_pan, int(max(0, ty) + h_f))

        offset_x = int(-tx) if tx < 0 else 0
        offset_y = int(-ty) if ty < 0 else 0

        if offset_x > 0 or offset_y > 0:
            new_w += offset_x
            new_h += offset_y

        large_canvas = np.zeros((new_h, new_w, 3), dtype=np.uint8)
        large_canvas[offset_y : offset_y + h_pan, offset_x : offset_x + w_pan] = (
            self.panorama
        )

        M_transform = np.array(
            [[1, 0, tx + offset_x], [0, 1, ty + offset_y]], dtype=np.float32
        )
        warped_frame = cv2.warpAffine(frame_cyl, M_transform, (new_w, new_h))
        warped_mask = cv2.warpAffine(frame_mask, M_transform, (new_w, new_h))

        mask_bool = warped_mask > 0
        large_canvas[mask_bool] = warped_frame[mask_bool]

        self.panorama = large_canvas
        self.added_frames_count += 1
        print(f"Added frame. New width: {self.panorama.shape[1]}px")
        return True

    def get_preview(self, max_width=1366):
        if self.panorama is None:
            return None
        scale_ratio = max_width / self.panorama.shape[1]
        if scale_ratio < 1.0:
            preview_w = int(self.panorama.shape[1] * scale_ratio)
            preview_h = int(self.panorama.shape[0] * scale_ratio)
            return cv2.resize(self.panorama, (preview_w, preview_h))
        return self.panorama.copy()

    def save_to_file(self):
        if self.panorama is not None:
            filename = f"panorama_{datetime.now().strftime('%H%M%S')}.png"
            cv2.imwrite(filename, self.panorama)
            print(f"Saved: {filename}")


def rotate_frame(img, angle):
    if angle == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    elif angle == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    elif angle == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img


def on_press(key):
    global current_action
    try:
        char = key.char.lower()
        if char in ["s", "r", "q"]:
            current_action = char
    except AttributeError:
        pass


def warp_cylinder(img, focal_length):
    h, w = img.shape[:2]
    y, x = np.indices((h, w))
    # middle of the image
    cx, cy = w / 2, h / 2

    theta = (x - cx) / focal_length
    h_ = (y - cy) / focal_length

    X = np.sin(theta)
    Y = h_
    Z = np.cos(theta)

    x_cyl = focal_length * X / Z + cx
    y_cyl = focal_length * Y / Z + cy

    mask = (x_cyl >= 0) & (x_cyl < w) & (y_cyl >= 0) & (y_cyl < h)

    maps_x = x_cyl.astype(np.float32)
    maps_y = y_cyl.astype(np.float32)

    cylindrical_img = cv2.remap(img, maps_x, maps_y, cv2.INTER_LINEAR)
    mask_img = (mask * 255).astype(np.uint8)
    return cylindrical_img, mask_img


def save_panorama(panorama_live):
    if panorama_live is not None:
        filename = f"panorama_{datetime.now().strftime('%H%M%S')}.png"
        cv2.imwrite(filename, panorama_live)
        print(f"Saved: {filename}")


def main(config):
    cap = cv2.VideoCapture(config["video_port"])
    if not cap.isOpened():
        print("Error: cannot open camera")
        return

    stitcher = CylindricalStitcher(focal_length=config["focal_length"])
    keyboard_controller = KeyboardController()
    keyboard_controller.start()

    frame_count = 0
    print("--- panorama 360 creator ---")
    print(" [S] - save panorama")
    print(" [R] - reset")
    print(" [Q] - exit")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1

        frame_oriented = rotate_frame(frame, config["camera_rotation"])
        frame_cyl, frame_mask = warp_cylinder(frame_oriented, config["focal_length"])

        if stitcher.panorama is None:
            stitcher.try_stitch(frame_cyl, frame_mask)
        elif frame_count % config["capture_interval"] == 0:
            stitcher.try_stitch(frame_cyl, frame_mask)

        # live preview camera
        cv2.putText(
            frame_cyl,
            f"frames: {stitcher.added_frames_count}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Live Camera", frame_cyl)

        # Live panorama preview
        preview_pano = stitcher.get_preview(max_width=1360)
        if preview_pano is not None:
            cv2.imshow("Live Panorama", preview_pano)

        cv2.waitKey(1)

        # Action handling
        action = keyboard_controller.get_action()
        if action == "q":
            print("Byee...")
            break
        elif action == "r":
            stitcher.reset()
            print("reset")
        elif action == "s":
            stitcher.save_to_file()

    keyboard_controller.stop()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    config = {
        "video_port": 1,  # video port
        "capture_interval": 15,  # capture interval
        "focal_length": 700,  # focal length typical [600-800]
        "camera_rotation": 270,  # camera rotation
    }
    main(config)
