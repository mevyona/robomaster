"""
Local Artificial Intelligence Vision Module for DJI RoboMaster S1.
Runs 100% on PC (YOLOv8 + OpenCV) with 0% browser lag.
Exclusively supported targets:
1. Person (only the closest person to the camera)
2. Bottle (bottle)
3. Soda can (can / cup)
- All other non-target objects are strictly ignored.
- Automatic turret tracking (wheels/chassis completely locked).
- Locking & Single-shot automatic infrared fire when target is centered.
- Sentry standby mode: continuous left/right turret sweep when idle.
"""

import argparse
import os
import socket
import sys
import time
import requests
import cv2
import numpy as np
from ultralytics import YOLO

# Supported targets
ALLOWED_TARGETS = {
    "1": ("person", "Person"),
    "2": ("bottle", "Bottle"),
    "3": ("can", "Soda can"),
    "4": ("all", "All 3 targets (Person, Bottle, Can)")
}

class SimpleObjectTracker:
    """
    Tracks individual objects across frames using centroid proximity and world angle.
    Remembers which targets have already been hit so the robot switches to another target.
    """
    def __init__(self, max_disappeared=25, hit_cooldown=30.0):
        self.next_id = 1
        self.objects = {}
        self.max_disappeared = max_disappeared
        self.hit_cooldown = hit_cooldown

    def update(self, detections, current_yaw=None):
        now = time.time()
        for obj_id, obj in list(self.objects.items()):
            if obj["is_hit"] and (now - obj["hit_time"] > self.hit_cooldown):
                obj["is_hit"] = False

        if len(detections) == 0:
            for obj_id in list(self.objects.keys()):
                self.objects[obj_id]["disappeared"] += 1
                if self.objects[obj_id]["disappeared"] > self.max_disappeared:
                    del self.objects[obj_id]
            return

        det_centroids = []
        for det in detections:
            x, y, w, h = det["bbox"]
            det_centroids.append((x + w / 2.0, y + h / 2.0))

        if len(self.objects) == 0:
            for i, det in enumerate(detections):
                self._register(det, det_centroids[i], current_yaw)
        else:
            obj_ids = list(self.objects.keys())
            matched_det_indices = set()
            matched_obj_ids = set()

            for obj_id in obj_ids:
                obj = self.objects[obj_id]
                best_dist = 999999
                best_idx = -1
                ox, oy = obj["centroid"]

                for i, det in enumerate(detections):
                    if i in matched_det_indices or det["category"] != obj["category"]:
                        continue
                    cx, cy = det_centroids[i]
                    dist = ((cx - ox) ** 2 + (cy - oy) ** 2) ** 0.5
                    if dist < 150 and dist < best_dist:
                        best_dist = dist
                        best_idx = i

                if best_idx != -1:
                    matched_det_indices.add(best_idx)
                    matched_obj_ids.add(obj_id)
                    obj["bbox"] = detections[best_idx]["bbox"]
                    obj["centroid"] = det_centroids[best_idx]
                    obj["disappeared"] = 0
                    if current_yaw is not None:
                        err_x = (det_centroids[best_idx][0] - 320.0) / 320.0
                        obj["world_yaw"] = current_yaw + (err_x * 48.0)
                    detections[best_idx]["track_id"] = obj_id
                    detections[best_idx]["is_hit"] = obj["is_hit"]

            for obj_id in obj_ids:
                if obj_id not in matched_obj_ids:
                    self.objects[obj_id]["disappeared"] += 1
                    if self.objects[obj_id]["disappeared"] > self.max_disappeared:
                        del self.objects[obj_id]

            for i, det in enumerate(detections):
                if i not in matched_det_indices:
                    self._register(det, det_centroids[i], current_yaw)

    def _register(self, det, centroid, current_yaw):
        obj_id = self.next_id
        self.next_id += 1
        world_yaw = 0
        if current_yaw is not None:
            err_x = (centroid[0] - 320.0) / 320.0
            world_yaw = current_yaw + (err_x * 48.0)

        is_hit = False
        hit_time = 0
        for old_obj in self.objects.values():
            if old_obj["is_hit"] and old_obj["category"] == det["category"]:
                if current_yaw is not None and abs(old_obj["world_yaw"] - world_yaw) < 18.0:
                    is_hit = True
                    hit_time = old_obj["hit_time"]
                    break

        self.objects[obj_id] = {
            "bbox": det["bbox"],
            "category": det["category"],
            "centroid": centroid,
            "disappeared": 0,
            "is_hit": is_hit,
            "hit_time": hit_time,
            "world_yaw": world_yaw
        }
        det["track_id"] = obj_id
        det["is_hit"] = is_hit

    def mark_hit(self, track_id):
        if track_id in self.objects:
            self.objects[track_id]["is_hit"] = True
            self.objects[track_id]["hit_time"] = time.time()

    def clear(self):
        self.objects.clear()
        self.next_id = 1

class LocalVisionTracker:
    def __init__(self, base_url="http://localhost:8080", target="can", conf=0.25, auto_fire=False, standby_mode=False, gui=False, rpi_ip=""):
        self.base_url = base_url.rstrip("/")
        self.target = self.normalize_target(target)
        self.conf_threshold = conf
        self.auto_fire = auto_fire
        self.standby_mode = standby_mode
        self.show_gui = gui
        self.rpi_ip = (rpi_ip or os.environ.get("RPI_LOG_IP", "") or os.environ.get("RPI_IP", "")).strip()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tracker = SimpleObjectTracker(max_disappeared=25, hit_cooldown=30.0)
        
        print("[+] Loading YOLOv8 nano vision model...")
        self.model = YOLO("yolov8n.pt")
        print("[✓] AI model ready.")

        # Human Pose & Keypoints model for Face Protection & Safe Torso Targeting
        print("[+] Loading YOLOv8 pose model for human safety & face avoidance...")
        try:
            self.pose_model = YOLO("yolov8n-pose.pt")
            print("[✓] Pose model ready (Human face protection enabled).")
        except Exception as e:
            print(f"[!] Warning: Could not load yolov8n-pose.pt ({e}), using anthropometric body model.")
            self.pose_model = None

        self.last_face_shift_log_time = 0

        self.last_pitch = 0
        self.last_yaw = 0
        self.is_moving = False
        self.last_sync_time = 0

        # Sentry Standby Mode & Physical Limit Detection
        self.standby_direction = 1  # 1 = right, -1 = left
        self.standby_speed = 22     # Smooth, surveillance-grade speed in deg/s (reduced from 48)
        self.standby_sweep_degrees = 480.0  # Full stop-to-stop rotation span (ensures full 360°+ coverage)
        self.standby_sweep_start = time.time()
        self.standby_sweep_start_yaw = None
        self.standby_last_yaw = None
        self.standby_last_yaw_time = time.time()
        self.standby_stall_count = 0
        # Dynamically calculated duration to ensure full rotation according to speed:
        self.standby_max_sweep_time = (self.standby_sweep_degrees / max(1.0, float(self.standby_speed))) + 2.0
        self.target_lost_grace_time = 0

        # Blind Spot / Dead Zone Detection & 360° Unwind
        self.is_unwinding = False
        self.unwind_direction = 0  # 1 = right, -1 = left
        self.unwind_speed = 52.0   # Speed during 360° unwind rotation
        self.unwind_start_time = 0
        self.unwind_min_time = 180.0 / self.unwind_speed  # Minimum time to clear dead zone (half-turn)
        self.unwind_timeout = (360.0 / self.unwind_speed) + 1.2  # Max time for full 360° turn
        self.tracking_stall_count = 0
        self.last_err_x = None

        # Target Lock & Auto-fire
        self.lock_start_time = None
        self.has_fired_for_current_target = False
        self.last_fire_time = 0
        self.just_fired_until = 0

        # State tracking for action logs
        self.target_tracked = False
        self.target_locked = False

    def log_action(self, category, message):
        """Appends a timestamped log entry to robot_actions.log, forwards to server & RPi"""
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        entry = f"[{ts}] [{category}] {message}\n"
        log_path = "robot_actions.log"
        if not os.path.exists(log_path) and os.path.exists("../robot_actions.log"):
            log_path = "../robot_actions.log"
        try:
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(entry)
        except Exception:
            pass

        # Forward to local RoboMaster Go server (which broadcasts to RPi if configured)
        server_ok = False
        try:
            r = requests.post(f"{self.base_url}/api/log", params={"cat": category, "msg": message}, timeout=0.15)
            if r.status_code == 200:
                server_ok = True
        except Exception:
            pass

        # Direct fallback UDP streaming to Raspberry Pi if server was unreachable
        if not server_ok and self.rpi_ip:
            try:
                self.udp_sock.sendto(entry.encode("utf-8"), (self.rpi_ip, 9999))
            except Exception:
                pass

    def normalize_target(self, target_str):
        if not target_str:
            return "can"
        t = target_str.lower().strip()
        if t in ["person", "personne", "humain", "human"]:
            return "person"
        if t in ["bottle", "bouteille"]:
            return "bottle"
        if t in ["can", "canette", "soda", "cup"]:
            return "can"
        if t in ["all", "toutes", "tout"]:
            return "all"
        return "can"

    def wait_for_server(self):
        """Verifies connection with the local RoboMaster server"""
        print(f"[+] Connecting to RoboMaster server ({self.base_url})...")
        while True:
            try:
                r = requests.get(f"{self.base_url}/api/status", timeout=1.5)
                if r.status_code == 200:
                    status = r.json()
                    bat = status.get("battery", 0)
                    print(f"[✓] Server connected! (Robot battery: {bat}%)")
                    break
            except Exception:
                pass
            print("    [!] Server not detected yet, retrying in 2 seconds...")
            time.sleep(2)

        # Sync RPi log target with server
        if self.rpi_ip:
            try:
                requests.post(f"{self.base_url}/api/rpi_log", params={"ip": self.rpi_ip}, timeout=1.0)
                print(f"[✓] Remote log streaming to Raspberry Pi configured: {self.rpi_ip}:9999")
            except Exception:
                pass

        # Synchronize target with server (prioritize Web Cockpit selection)
        server_target = self.get_server_target()
        if server_target:
            self.target = server_target
            print(f"[✓] Initial target from Web Cockpit: {self.target.upper()}")
        elif self.target:
            self.set_server_target(self.target)

    def get_server_target(self):
        """Reads active target requested from Web Cockpit"""
        try:
            r = requests.get(f"{self.base_url}/api/target", timeout=0.4)
            if r.status_code == 200:
                data = r.json()
                return self.normalize_target(data.get("target", "person"))
        except Exception:
            pass
        return None

    def get_telemetry(self):
        """Fetches attitude telemetry (yaw, pitch, yaw_speed) from server"""
        try:
            r = requests.get(f"{self.base_url}/api/status", timeout=0.25)
            if r.status_code == 200:
                data = r.json()
                yaw = data.get("yaw")
                pitch = data.get("pitch")
                yspd = data.get("yaw_speed")
                return yaw, pitch, yspd
        except Exception:
            pass
        return None, None, None

    def set_server_target(self, target):
        """Updates active target on the server for Web Cockpit display"""
        try:
            requests.post(
                f"{self.base_url}/api/target",
                params={"target": target},
                timeout=0.4
            )
        except Exception:
            pass

    def sync_server_config(self):
        """Synchronizes target, auto-fire and standby toggles with Web Cockpit"""
        now = time.time()
        if now - self.last_sync_time < 0.8:
            return

        self.last_sync_time = now

        # Synchronize target
        server_target = self.get_server_target()
        if server_target and server_target != self.target:
            old = self.target
            self.target = server_target
            self.tracker.clear()
            print(f"\n[🎯] TARGET SWITCHED VIA COCKPIT: '{old.upper()}' -> '{self.target.upper()}'")
            self.log_action("CONFIG", f"AI target changed via Web Cockpit: '{old.upper()}' -> '{self.target.upper()}' (Hit history cleared)")
            self.stop_gimbal()
            self.lock_start_time = None
            self.has_fired_for_current_target = False
            self.target_tracked = False
            self.target_locked = False

        # Synchronize auto-fire
        try:
            r = requests.get(f"{self.base_url}/api/autofire", timeout=0.3)
            if r.status_code == 200:
                af = r.json().get("auto_fire", False)
                if af != self.auto_fire:
                    self.auto_fire = af
                    self.log_action("CONFIG", f"Auto-fire setting synchronized: {self.auto_fire}")
        except Exception:
            pass

        # Synchronize standby sentry mode
        try:
            r = requests.get(f"{self.base_url}/api/standby", timeout=0.3)
            if r.status_code == 200:
                st = r.json().get("standby", False)
                if st != self.standby_mode:
                    self.standby_mode = st
                    self.log_action("CONFIG", f"Sentry standby mode synchronized: {self.standby_mode}")
        except Exception:
            pass

    def fetch_frame(self):
        """Fetches JPEG frame from local camera snapshot endpoint"""
        try:
            resp = requests.get(f"{self.base_url}/snapshot", timeout=1.0)
            if resp.status_code == 200 and len(resp.content) > 0:
                arr = np.frombuffer(resp.content, np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                return frame
        except Exception:
            pass
        return None

    def post_detections(self, detections):
        """Sends detections to server for browser HUD overlay"""
        try:
            requests.post(
                f"{self.base_url}/api/detections",
                json=detections,
                timeout=0.2
            )
        except Exception:
            pass

    def fire(self):
        """Triggers 1 infrared shot via robot API"""
        try:
            requests.post(f"{self.base_url}/api/fire", timeout=0.4)
        except Exception:
            pass

    def send_gimbal(self, pitch, yaw):
        """Sends smooth rotation speed commands to the turret"""
        if pitch == 0 and yaw == 0 and not self.is_moving:
            return

        try:
            requests.post(
                f"{self.base_url}/api/gimbal",
                params={"pitch": pitch, "yaw": yaw},
                timeout=0.3
            )
            self.last_pitch = pitch
            self.last_yaw = yaw
            self.is_moving = (pitch != 0 or yaw != 0)
        except Exception:
            pass

    def stop_gimbal(self):
        """Immediately halts turret rotation"""
        self.send_gimbal(0, 0)
        self.is_moving = False

    def analyze_human_body(self, frame, person_box):
        """
        Analyzes human body to identify face exclusion zone and calculate safe torso target.
        Ensures the robot never shoots in faces.
        Returns:
            face_box: [fx, fy, fw, fh]
            safe_torso_point: (tx, ty)
            aiming_at_face: bool (true if crosshair is inside face exclusion zone)
            has_safe_body_target: bool (true if safe body area is visible)
        """
        bx, by, bw, bh = person_box
        frame_h, frame_w = frame.shape[:2]

        face_box = None
        safe_torso_point = None

        # Try keypoints extraction via YOLOv8-pose
        if self.pose_model is not None and bw > 25 and bh > 25:
            try:
                x1 = max(0, bx)
                y1 = max(0, by)
                x2 = min(frame_w, bx + bw)
                y2 = min(frame_h, by + bh)
                crop = frame[y1:y2, x1:x2]

                if crop.size > 0:
                    pose_results = self.pose_model(crop, imgsz=256, verbose=False)[0]
                    if pose_results.keypoints is not None and len(pose_results.keypoints.xy) > 0:
                        kpts = pose_results.keypoints.xy[0].cpu().numpy()
                        confs = pose_results.keypoints.conf[0].cpu().numpy() if pose_results.keypoints.conf is not None else np.ones(17)

                        # Shift crop coordinates back to full frame
                        kpts[:, 0] += x1
                        kpts[:, 1] += y1

                        # Face keypoints: nose (0), left_eye (1), right_eye (2), left_ear (3), right_ear (4)
                        face_pts = [kpts[i] for i in range(5) if confs[i] > 0.30]
                        if len(face_pts) > 0:
                            f_xs = [p[0] for p in face_pts]
                            f_ys = [p[1] for p in face_pts]
                            min_fx, max_fx = min(f_xs), max(f_xs)
                            min_fy, max_fy = min(f_ys), max(f_ys)
                            fw = max(bw * 0.40, (max_fx - min_fx) * 1.8)
                            fh = max(bh * 0.22, (max_fy - min_fy) * 2.0)
                            fcx = (min_fx + max_fx) / 2.0
                            fcy = (min_fy + max_fy) / 2.0
                            face_box = [int(fcx - fw / 2.0), int(fcy - fh * 0.4), int(fw), int(fh)]

                        # Safe torso target using shoulders (5, 6) and hips (11, 12)
                        s_left_conf, s_right_conf = confs[5], confs[6]
                        if s_left_conf > 0.30 and s_right_conf > 0.30:
                            sx = (kpts[5][0] + kpts[6][0]) / 2.0
                            sy = (kpts[5][1] + kpts[6][1]) / 2.0

                            h_left_conf, h_right_conf = confs[11], confs[12]
                            if h_left_conf > 0.30 and h_right_conf > 0.30:
                                hx = (kpts[11][0] + kpts[12][0]) / 2.0
                                hy = (kpts[11][1] + kpts[12][1]) / 2.0
                                safe_torso_point = (float(sx * 0.65 + hx * 0.35), float(sy * 0.65 + hy * 0.35))
                            else:
                                safe_torso_point = (float(sx), float(sy + min(bh * 0.22, 55)))
            except Exception:
                pass

        # Anthropometric fallback if pose detection was partial or unavailable
        if face_box is None:
            fw = int(bw * 0.55)
            fh = int(bh * 0.26)
            fx = int(bx + (bw - fw) / 2.0)
            fy = int(by)
            face_box = [fx, fy, fw, fh]

        if safe_torso_point is None:
            safe_torso_point = (float(bx + bw * 0.5), float(by + bh * 0.52))

        # Check if crosshair (center of camera frame) is currently aiming at face
        crosshair_x = frame_w / 2.0
        crosshair_y = frame_h / 2.0
        fx, fy, fw, fh = face_box
        margin = 12
        aiming_at_face = (
            (fx - margin <= crosshair_x <= fx + fw + margin) and
            (fy - margin <= crosshair_y <= fy + fh + margin)
        )

        tx, ty = safe_torso_point
        has_safe_body_target = (ty < frame_h - 10) and (by + bh - (fy + fh) > 25)

        return face_box, safe_torso_point, aiming_at_face, has_safe_body_target

    def calculate_tracking(self, frame_w, frame_h, target_box, target_point=None):
        """
        Calculates turret tracking velocity corrections with progressive deceleration:
        - The closer the crosshair gets to the target, the more the speed smoothly slows down for precise micro-aiming.
        - Pitch: vertical (+ = up, - = down)
        - Yaw: horizontal (+ = right, - = left)
        If target_point is provided, calculates errors relative to that point (e.g. torso).
        """
        if target_point is not None:
            target_cx, target_cy = target_point
        else:
            x, y, w, h = target_box
            target_cx = x + w / 2.0
            target_cy = y + h / 2.0

        err_x = (target_cx - (frame_w / 2.0)) / (frame_w / 2.0)
        err_y = (target_cy - (frame_h / 2.0)) / (frame_h / 2.0)

        deadband = 0.04

        # Yaw speed (Horizontal) with smooth progressive deceleration
        abs_err_x = abs(err_x)
        yaw_speed = 0
        if abs_err_x > deadband:
            ratio_x = np.clip((abs_err_x - deadband) / (1.0 - deadband), 0.0, 1.0)
            # Far -> up to 48 deg/s; Close -> smoothly slows down to 7 deg/s for precision micro-aiming
            speed_x = 7.0 + (48.0 - 7.0) * (ratio_x ** 1.3)
            yaw_speed = int(np.copysign(speed_x, err_x))

        # Pitch speed (Vertical) with smooth progressive deceleration
        abs_err_y = abs(err_y)
        pitch_speed = 0
        if abs_err_y > deadband:
            ratio_y = np.clip((abs_err_y - deadband) / (1.0 - deadband), 0.0, 1.0)
            # Far -> up to 35 deg/s; Close -> smoothly slows down to 6 deg/s for precision micro-aiming
            speed_y = 6.0 + (35.0 - 6.0) * (ratio_y ** 1.3)
            pitch_speed = int(np.copysign(speed_y, -err_y))

        return pitch_speed, yaw_speed, err_x, err_y

    def matches_target(self, category):
        if self.target == "all":
            return True
        if self.target == "can" and category in ["can", "canette", "cup"]:
            return True
        if self.target == "bottle" and category in ["bottle", "bouteille"]:
            return True
        if self.target == "person" and category in ["person", "humain", "human"]:
            return True
        return self.target == category

    def run(self):
        self.wait_for_server()

        print("\n" + "=" * 60)
        print("   DJI ROBOMASTER S1 AI VISION: PERSON / BOTTLE / CAN")
        print("=" * 60)
        print(f"[•] Active Target     : {self.target.upper()}")
        print(f"[•] Confidence Thresh : {int(self.conf_threshold * 100)}%")
        print(f"[•] Automatic Fire    : {'ENABLED' if self.auto_fire else 'DISABLED'}")
        print(f"[•] Sentry Standby    : {'ENABLED' if self.standby_mode else 'DISABLED'}")
        print(f"[•] Web Cockpit       : {self.base_url}")
        if self.show_gui:
            print("[•] OpenCV Window     : Enabled (Press 'q' to quit)")
        print("[•] Stop              : Press Ctrl+C in this terminal.\n")

        self.log_action("AI", f"Local AI vision started (Target: {self.target.upper()}, Conf: {int(self.conf_threshold * 100)}%, AutoFire: {self.auto_fire}, Standby: {self.standby_mode})")

        last_log_time = 0

        try:
            while True:
                loop_start = time.time()

                # Synchronize settings with Web Cockpit
                self.sync_server_config()

                frame = self.fetch_frame()
                if frame is None:
                    time.sleep(0.04)
                    continue

                frame_h, frame_w = frame.shape[:2]

                # YOLOv8 nano inference
                results = self.model(frame, verbose=False, conf=self.conf_threshold)[0]

                # 1. Filter detections strictly to allowed categories
                person_boxes = []
                bottle_boxes = []
                can_boxes = []

                for box in results.boxes:
                    cls_id = int(box.cls[0].item())
                    label = self.model.names[cls_id]
                    score = float(box.conf[0].item())
                    xyxy = box.xyxy[0].tolist()

                    bx = int(xyxy[0])
                    by = int(xyxy[1])
                    bw = int(xyxy[2] - xyxy[0])
                    bh = int(xyxy[3] - xyxy[1])
                    area = bw * bh

                    if label == "person":
                        person_boxes.append({
                            "category": "person",
                            "label": "person",
                            "score": round(score, 2),
                            "bbox": [bx, by, bw, bh],
                            "area": area
                        })
                    elif label == "bottle":
                        bottle_boxes.append({
                            "category": "bottle",
                            "label": "bottle",
                            "score": round(score, 2),
                            "bbox": [bx, by, bw, bh],
                            "area": area
                        })
                    elif label in ["cup", "can"]:
                        can_boxes.append({
                            "category": "can",
                            "label": "can",
                            "score": round(score, 2),
                            "bbox": [bx, by, bw, bh],
                            "area": area
                        })

                # 2. Keep all detected persons, bottles, and cans (any target is eligible for engagement)
                filtered_detections = []
                if len(person_boxes) > 0:
                    filtered_detections.extend(person_boxes)

                # Bottles & Cans: Keep all detected bottles and cans for tracking and sequential target switching
                if len(bottle_boxes) > 0:
                    filtered_detections.extend(bottle_boxes)

                if len(can_boxes) > 0:
                    filtered_detections.extend(can_boxes)

                # Fetch current attitude telemetry before tracker update
                current_yaw, current_pitch, current_yaw_speed = self.get_telemetry()

                # Track objects and match with hit history
                self.tracker.update(filtered_detections, current_yaw)

                # 3. Select best un-hit target to track
                best_target_box = None
                best_target_score = -1.0
                best_target_label = ""
                best_target_category = ""
                best_target_track_id = None

                raw_detections = []
                for det in filtered_detections:
                    is_target = self.matches_target(det["category"])
                    det["is_target"] = is_target
                    det["is_locked"] = False
                    det["just_fired"] = False
                    is_hit = det.get("is_hit", False)

                    # Human safety and face protection metadata
                    if det["category"] == "person":
                        f_box, s_torso, aim_face, has_body = self.analyze_human_body(frame, det["bbox"])
                        det["face_box"] = f_box
                        det["aim_point"] = [int(s_torso[0]), int(s_torso[1])]
                        det["aiming_at_face"] = aim_face
                        det["has_safe_body_target"] = has_body
                        det["face_protected"] = True
                    else:
                        bx, by, bw, bh = det["bbox"]
                        det["face_box"] = None
                        det["aim_point"] = [int(bx + bw / 2.0), int(by + bh / 2.0)]
                        det["aiming_at_face"] = False
                        det["has_safe_body_target"] = True
                        det["face_protected"] = False

                    raw_detections.append(det)

                    # Target switching: Only UN-HIT targets are eligible for selection!
                    if is_target and not is_hit:
                        bx, by, bw, bh = det["bbox"]
                        cx, cy = bx + bw / 2.0, by + bh / 2.0
                        dist_to_crosshair = ((cx - frame_w / 2.0) ** 2 + (cy - frame_h / 2.0) ** 2) ** 0.5
                        if best_target_score < 0 or dist_to_crosshair < best_target_score:
                            best_target_score = dist_to_crosshair
                            best_target_box = det["bbox"]
                            best_target_label = det["label"]
                            best_target_category = det["category"]
                            best_target_track_id = det.get("track_id")

                is_target_locked = False
                now = time.time()
                just_fired = (now < self.just_fired_until)

                # Handle active 360° UNWIND (Dead zone / Blind spot recovery)
                if self.is_unwinding:
                    unwind_duration = now - self.unwind_start_time
                    err_x = 0.0
                    if best_target_box is not None:
                        _, _, err_x, _ = self.calculate_tracking(frame_w, frame_h, best_target_box)

                    # Check if unwind can conclude: target re-acquired from open angle or timeout reached
                    if unwind_duration >= self.unwind_min_time and best_target_box is not None and abs(err_x) < 0.35:
                        self.is_unwinding = False
                        self.log_action("UNWIND", "Target re-acquired from open angle -> Resuming active tracking")
                        print("\n[✓ 360° COMPLETE] Target re-acquired from optimal angle -> Resuming tracking!")
                    elif unwind_duration >= self.unwind_timeout:
                        self.is_unwinding = False
                        self.log_action("UNWIND", "360° rotation completed -> Resuming normal tracking")
                        print("\n[✓ 360° COMPLETE] Full rotation done -> Resuming tracking!")
                    else:
                        self.send_gimbal(0, int(self.unwind_speed * self.unwind_direction))
                        if now - last_log_time > 2.0:
                            dir_str = "LEFT ⬅" if self.unwind_direction < 0 else "RIGHT ➔"
                            print(f"[🔄 360° UNWIND] Rotating {dir_str} to reach target dead zone from the opposite angle...")
                            last_log_time = now
                        time.sleep(0.04)
                        continue

                # 4. Tracking and firing
                if best_target_box is not None:
                    self.target_lost_grace_time = now + 0.6

                    if not self.target_tracked:
                        self.target_tracked = True
                        self.log_action("VISION", f"Target '{best_target_label.upper()}' detected (Area: {int(best_target_score)} px)")

                    # Determine target point: for humans, strictly aim at Torso/Chest to avoid faces
                    target_point = None
                    is_person_target = (best_target_category == "person")
                    aiming_at_face = False
                    has_safe_body = True

                    if is_person_target:
                        face_box, safe_torso_point, aiming_at_face, has_safe_body = self.analyze_human_body(frame, best_target_box)
                        target_point = safe_torso_point

                        # Safety Interlock: If crosshair is currently aiming at face, change targeted body part immediately
                        if aiming_at_face:
                            if now - self.last_face_shift_log_time > 1.8:
                                print("\n[⚠️ SAFETY INTERLOCK] Aiming at human face -> Changing targeted body part to CHEST/TORSO (Fire blocked!)")
                                self.log_action("SAFETY", "Aiming at human face -> Retargeted aim to CHEST/TORSO (Face shot prevented)")
                                self.last_face_shift_log_time = now

                    pitch, yaw, err_x, err_y = self.calculate_tracking(frame_w, frame_h, best_target_box, target_point=target_point)

                    # BLIND SPOT / DEAD ZONE DETECTION:
                    # Detect if servomotor reached physical limit while target is still off-center towards that limit
                    is_stuck_right = False
                    is_stuck_left = False

                    if current_yaw is not None:
                        if current_yaw >= 230.0 and err_x > 0.10:
                            is_stuck_right = True
                        elif current_yaw <= -230.0 and err_x < -0.10:
                            is_stuck_left = True

                    if not is_stuck_right and not is_stuck_left:
                        if err_x > 0.15 and self.last_err_x is not None and (err_x >= self.last_err_x - 0.02):
                            self.tracking_stall_count += 1
                            if self.tracking_stall_count >= 10:
                                is_stuck_right = True
                        elif err_x < -0.15 and self.last_err_x is not None and (err_x <= self.last_err_x + 0.02):
                            self.tracking_stall_count += 1
                            if self.tracking_stall_count >= 10:
                                is_stuck_left = True
                        else:
                            self.tracking_stall_count = max(0, self.tracking_stall_count - 1)

                    self.last_err_x = err_x

                    # If stuck at right limit, turn 360° LEFT to reach target from other side
                    if is_stuck_right:
                        self.is_unwinding = True
                        self.unwind_direction = -1
                        self.unwind_start_time = now
                        self.tracking_stall_count = 0
                        self.log_action("UNWIND", "Target in blind spot at MAX RIGHT limit -> Initiating 360° rotation to the LEFT")
                        print("\n[🔄 BLIND SPOT DETECTED] Target in servomotor dead zone at MAX RIGHT -> Unwinding 360° to the LEFT!")
                        self.send_gimbal(0, -52)
                        time.sleep(0.04)
                        continue
                    # If stuck at left limit, turn 360° RIGHT to reach target from other side
                    elif is_stuck_left:
                        self.is_unwinding = True
                        self.unwind_direction = 1
                        self.unwind_start_time = now
                        self.tracking_stall_count = 0
                        self.log_action("UNWIND", "Target in blind spot at MAX LEFT limit -> Initiating 360° rotation to the RIGHT")
                        print("\n[🔄 BLIND SPOT DETECTED] Target in servomotor dead zone at MAX LEFT -> Unwinding 360° to the RIGHT!")
                        self.send_gimbal(0, 52)
                        time.sleep(0.04)
                        continue

                    # Normal tracking command
                    self.send_gimbal(pitch, yaw)

                    # Centered criteria: target within 10% deadband
                    # STRICT SAFETY RULE: Cannot lock or fire if aiming at face or no safe body target available
                    is_centered = (abs(err_x) < 0.10 and abs(err_y) < 0.10)
                    if is_person_target and (aiming_at_face or not has_safe_body):
                        is_centered = False

                    if is_centered:
                        if self.lock_start_time is None:
                            self.lock_start_time = now

                        lock_duration = now - self.lock_start_time
                        if lock_duration >= 0.35:
                            is_target_locked = True
                            if not self.target_locked:
                                self.target_locked = True
                                lock_target_name = "TORSO of " + best_target_label.upper() if is_person_target else best_target_label.upper()
                                self.log_action("LOCK", f"Target '{lock_target_name}' locked at crosshair center (Face clear)")

                            # Trigger single shot fire
                            if self.auto_fire and not self.has_fired_for_current_target and (now - self.last_fire_time > 2.0):
                                fire_desc = f"TORSO of '{best_target_label.upper()}' (Face protected)" if is_person_target else f"target '{best_target_label.upper()}'"
                                track_str = f" #{best_target_track_id}" if best_target_track_id else ""
                                print(f"\n[💥💥 BOOM!] TARGET {fire_desc}{track_str} LOCKED AT CENTER -> REAL SHOT EXECUTED!")
                                self.log_action("FIRE", f"Automatic real shot triggered on {fire_desc}{track_str}")
                                self.fire()


                                # Mark target as HIT so the robot switches to another target
                                if best_target_track_id is not None:
                                    self.tracker.mark_hit(best_target_track_id)
                                    self.log_action("TARGET", f"Target '{best_target_label.upper()}{track_str}' hit -> Switching to next available target")
                                    print(f"[🎯 SWITCH] Target{track_str} hit! Switching to next target...")

                                self.last_fire_time = now
                                self.just_fired_until = now + 1.2
                                just_fired = True
                                self.lock_start_time = None
                                self.target_locked = False
                                self.has_fired_for_current_target = False
                    else:
                        self.lock_start_time = None
                        self.target_locked = False
                        if now - self.last_fire_time > 3.0:
                            self.has_fired_for_current_target = False

                    if now - last_log_time > 1.2:
                        state_str = "🔒 LOCK!" if is_target_locked else f"TRACKING (P:{pitch}, Y:{yaw})"
                        target_info = f"{best_target_label.upper()} [AIM: TORSO]" if is_person_target else best_target_label.upper()
                        print(f"[🎯] {target_info} detected -> Turret: {state_str}")
                        last_log_time = now
                else:
                    self.lock_start_time = None
                    self.has_fired_for_current_target = False
                    self.tracking_stall_count = 0
                    self.last_err_x = None

                    if self.target_tracked:
                        self.target_tracked = False
                        self.target_locked = False
                        self.log_action("VISION", "Target lost from view")

                    # Sentry Standby Mode: sweep until physical limit is reached, then reverse
                    if self.standby_mode:
                        if now < self.target_lost_grace_time:
                            self.stop_gimbal()
                        else:
                            limit_reached = False
                            time_in_sweep = now - self.standby_sweep_start

                            # 1. Telemetry limit check
                            if current_yaw is not None:
                                if self.standby_sweep_start_yaw is None:
                                    self.standby_sweep_start_yaw = current_yaw

                                # Detect physical mechanical limits (~ +/- 235 deg)
                                if self.standby_direction > 0 and current_yaw >= 235.0:
                                    limit_reached = True
                                elif self.standby_direction < 0 and current_yaw <= -235.0:
                                    limit_reached = True
                                elif time_in_sweep > 1.5 and self.standby_last_yaw is not None:
                                    # Stall detection: yaw stopped moving against physical stop
                                    if abs(current_yaw - self.standby_last_yaw) < 0.4:
                                        self.standby_stall_count += 1
                                        if self.standby_stall_count >= 4:
                                            limit_reached = True
                                    else:
                                        self.standby_stall_count = 0

                            # 2. Dynamic sweep time adaptation:
                            # Sweep time is dynamically calculated: (sweep_degrees / standby_speed) + margin
                            # Ensures the turret always has enough time to cover 360 deg or the full mechanical span at ANY speed
                            dynamic_sweep_time = (self.standby_sweep_degrees / max(1.0, float(self.standby_speed))) + 1.5
                            if not limit_reached and time_in_sweep >= dynamic_sweep_time:
                                limit_reached = True

                            # If limit reached, reverse direction immediately!
                            if limit_reached:
                                was_right = (self.standby_direction > 0)
                                self.standby_direction *= -1
                                self.standby_sweep_start = now
                                self.standby_sweep_start_yaw = current_yaw
                                self.standby_stall_count = 0
                                stop_name = "MAX RIGHT" if was_right else "MAX LEFT"
                                new_side = "LEFT ⬅" if self.standby_direction < 0 else "RIGHT ➔"
                                self.log_action("STANDBY", f"Servomotor reached physical limit at {stop_name} -> Reversing sweep towards {new_side}")
                                print(f"\n[📡 LIMIT DETECTED] Servomotor reached {stop_name} limit -> Reversing sweep towards {new_side}!")

                            if now - self.standby_last_yaw_time > 0.2:
                                self.standby_last_yaw = current_yaw
                                self.standby_last_yaw_time = now

                            sweep_yaw = int(self.standby_speed * self.standby_direction)
                            self.send_gimbal(0, sweep_yaw)

                            if now - last_log_time > 3.0:
                                dir_str = "RIGHT ➔" if self.standby_direction > 0 else "⬅ LEFT"
                                yaw_info = f" (Yaw: {current_yaw:+.1f}°)" if current_yaw is not None else ""
                                print(f"[📡 STANDBY 360°] Sentry turret sweeping {dir_str}{yaw_info}... searching for target")
                                last_log_time = now
                    else:
                        self.stop_gimbal()

                    if len(raw_detections) > 0 and now - last_log_time > 2.0:
                        detected_names = ", ".join([f"{d['label']} ({int(d['score']*100)}%)" for d in raw_detections])
                        print(f"[👁️] Non-target object(s) detected: {detected_names} | Required target: {self.target.upper()}")
                        last_log_time = now

                # Mark lock and fired flags for Web HUD
                for det in raw_detections:
                    if best_target_box is not None and det["bbox"] == list(best_target_box):
                        det["is_locked"] = is_target_locked
                        det["just_fired"] = just_fired

                # Send detections to web dashboard
                self.post_detections(raw_detections)

                # Optional OpenCV debug window
                if self.show_gui:
                    annotated = frame.copy()
                    for det in raw_detections:
                        bx, by, bw, bh = det["bbox"]
                        is_hit = det.get("is_hit", False)

                        if is_hit:
                            # Hit / eliminated target
                            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 140, 255), 2)
                            cv2.putText(annotated, f"HIT #{det.get('track_id', '')}", (bx, max(15, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)
                        else:
                            color = (0, 255, 0) if det.get("is_target") else (255, 100, 0)
                            if det.get("is_locked"):
                                color = (0, 0, 255)
                            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), color, 2)
                            
                            # Face exclusion zone
                            fb = det.get("face_box")
                            if fb:
                                fx, fy, fw, fh = fb
                                cv2.rectangle(annotated, (fx, fy), (fx + fw, fy + fh), (0, 140, 255), 2)
                                cv2.putText(annotated, "NO-FIRE: FACE", (fx, max(15, fy - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 1)
                            
                            # Safe aim point
                            ap = det.get("aim_point")
                            if ap:
                                ax, ay = ap
                                cv2.circle(annotated, (ax, ay), 6, (255, 255, 0), -1)
                                cv2.putText(annotated, "TORSO TARGET", (ax + 10, ay + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

                    if best_target_category == "person" and aiming_at_face:
                        cv2.putText(annotated, "SAFETY ACTIVE: FACE PROTECTED -> RETARGETING TORSO", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                    cv2.imshow("DJI RoboMaster S1 - Local AI", annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                # ~15 FPS frequency
                elapsed = time.time() - loop_start
                sleep_time = max(0.01, 0.06 - elapsed)
                time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[+] Stopping local AI...")
        finally:
            self.log_action("AI", "Local AI vision stopped, turret halted")
            self.stop_gimbal()
            if self.show_gui:
                cv2.destroyAllWindows()
            print("[✓] Turret halted and AI stopped.")

def main():
    parser = argparse.ArgumentParser(description="DJI RoboMaster S1 - Local AI Vision & Turret Tracking")
    parser.add_argument("--url", default="http://localhost:8080", help="Base URL of local RoboMaster server")
    parser.add_argument("--target", default="can", help="Target: can, person, bottle, all (default: can)")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold (default: 0.25)")
    parser.add_argument("--auto-fire", action="store_true", help="Enable automatic fire on lock (default: False)")
    parser.add_argument("--standby", action="store_true", help="Enable sentry sweep when no target is present (default: False)")
    parser.add_argument("--gui", action="store_true", help="Show local OpenCV window with bounding boxes")
    parser.add_argument("--rpi-ip", default=os.environ.get("RPI_LOG_IP", os.environ.get("RPI_IP", "")), help="Raspberry Pi IP address for remote log streaming (UDP :9999)")
    args = parser.parse_args()

    tracker = LocalVisionTracker(
        base_url=args.url,
        target=args.target,
        conf=args.conf,
        auto_fire=args.auto_fire,
        standby_mode=args.standby,
        gui=args.gui,
        rpi_ip=args.rpi_ip
    )
    tracker.run()

if __name__ == "__main__":
    main()
