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
        # Używamy FLANN dla szybszego i dokładniejszego dopasowywania przy dużych zestawach cech
        index_params = dict(algorithm=1, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.panorama = None
        self.added_frames_count = 0
        self.is_saved_360 = False

        # Teoretyczna szerokość pełnego obrotu 360 stopni
        self.target_width_360 = int(2 * np.pi * self.focal_length)

        # Przechowywanie punktów kluczowych pierwszej klatki
        self.first_frame_kp = None
        self.first_frame_des = None

    def reset(self):
        self.panorama = None
        self.added_frames_count = 0
        self.is_saved_360 = False
        self.first_frame_kp = None
        self.first_frame_des = None
        print("[System] Resetowanie panoramy...")

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

    def check_loop_closure(self, frame_cyl):
        """Sprawdza powrót do startu po przekroczeniu 50% szerokości przy użyciu Homografii."""
        if self.panorama is None or self.first_frame_des is None:
            return False

        current_width = self.panorama.shape[1]
        half_width_threshold = self.target_width_360 * 0.5

        # WARUNEK 1: Sprawdzamy dopiero, gdy jesteśmy przynajmniej w połowie oczekiwanej szerokości
        if current_width < half_width_threshold:
            return False

        kp_frame, des_frame = self.finder.detectAndCompute(frame_cyl, None)
        if des_frame is None:
            return False

        # Dopasowanie przy użyciu FLANN (stabilniejsze niż podstawowy BFMatcher)
        matches = self.flann.knnMatch(des_frame, self.first_frame_des, k=2)
        good_matches = []
        for match in matches:
            if len(match) == 2:
                m, n = match
                # Wybór najbardziej unikalnych punktów (test Ratio Lowe'a)
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)

        # WARUNEK 2: Jeśli mamy potencjalne punkty wspólne, weryfikujemy je Homografią
        if len(good_matches) >= 15:
            src_pts = np.float32(
                [kp_frame[m.queryIdx].pt for m in good_matches]
            ).reshape(-1, 1, 2)
            dst_pts = np.float32(
                [self.first_frame_kp[m.trainIdx].pt for m in good_matches]
            ).reshape(-1, 1, 2)

            # Homografia (findHomography + RANSAC) wybacza drobne różnice w kącie patrzenia kamery
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            if H is not None:
                num_inliers = np.sum(mask)
                # Jeżeli poprawnie przetłumaczono geometrię dla min. 12 punktów - mamy to!
                if num_inliers >= 12:
                    print(
                        f"\n[Loop Closure] Znaleziono dopasowanie perspektywiczne! Poprawne punkty (inliers): {num_inliers}"
                    )
                    return True
        return False

    def try_stitch(self, frame_cyl, frame_mask):
        # Inicjalizacja pierwszej klatki
        if self.panorama is None:
            self.panorama = frame_cyl.copy()
            self.added_frames_count = 1
            # Konwertujemy deskryptory na float32 (wymóg dla FLANN Matchera)
            kp, des = self.finder.detectAndCompute(frame_cyl, None)
            if des is not None:
                self.first_frame_kp = kp
                self.first_frame_des = des.astype(np.float32)
            return True

        # Sprawdzamy pętlę przed modyfikacją panoramy
        if self.is_saved_360 is False and self.check_loop_closure(frame_cyl):
            self.is_saved_360 = True

        # Konwersja deskryptorów panoramy i klatki na float32 do dopasowania standardowego
        kp1, des1 = self.finder.detectAndCompute(self.panorama, None)
        kp2, des2 = self.finder.detectAndCompute(frame_cyl, None)

        if des1 is None or des2 is None:
            return False

        des1_f = des1.astype(np.float32)
        des2_f = des2.astype(np.float32)

        matches = self.flann.knnMatch(des2_f, des1_f, k=2)
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

        # Informacja o postępie szerokości w konsoli
        progress_pct = (self.panorama.shape[1] / self.target_width_360) * 100
        print(
            f"Added frame. Width: {self.panorama.shape[1]}px ({progress_pct:.1f}% of 360° target)"
        )

        # Zapis automatyczny po wykryciu pętli
        if self.is_saved_360 is True:
            self.save_to_file(prefix="panorama_360_hybrid")
            self.is_saved_360 = "done"

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

    def save_to_file(self, prefix="panorama"):
        if self.panorama is not None:
            filename = f"{prefix}_{datetime.now().strftime('%H%M%S')}.png"
            cv2.imwrite(filename, self.panorama)
            print(f"====== AUTO-SAVE SUCCESFUL: {filename} ======")


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
        "video_port": 2,  # video port
        "capture_interval": 15,  # capture interval
        "focal_length": 700,  # focal length typical [600-800]
        "camera_rotation": 90,  # camera rotation
    }
    main(config)
