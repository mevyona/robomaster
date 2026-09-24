import requests
import time

class RoboMaster:
    """
    Python helper library to control the DJI RoboMaster S1 gimbal/camera
    and interact with video feeds and vision detection without moving chassis wheels.
    """
    def __init__(self, base_url="http://localhost:8080"):
        self.base_url = base_url.rstrip("/")

    def set_gimbal_speed(self, pitch=0, yaw=0):
        """
        Sets gimbal rotational speed:
        - pitch: vertical speed (+ = up, - = down)
        - yaw: horizontal speed (+ = right, - = left)
        """
        requests.post(f"{self.base_url}/api/gimbal", params={"pitch": pitch, "yaw": yaw})

    def gimbal(self, pitch=0, yaw=0, duration=0.5):
        """
        Rotates the gimbal/camera for a specific duration:
        - pitch: vertical speed (+ = up, - = down)
        - yaw: horizontal speed (+ = right, - = left)
        - duration: movement duration in seconds
        """
        self.set_gimbal_speed(pitch, yaw)
        time.sleep(duration)
        self.set_gimbal_speed(0, 0)

    def look_up(self, speed=30, duration=0.5):
        """Tilts the camera upward"""
        self.gimbal(pitch=speed, yaw=0, duration=duration)

    def look_down(self, speed=30, duration=0.5):
        """Tilts the camera downward"""
        self.gimbal(pitch=-speed, yaw=0, duration=duration)

    def look_left(self, speed=35, duration=0.5):
        """Pans the camera left"""
        self.gimbal(pitch=0, yaw=-speed, duration=duration)

    def look_right(self, speed=35, duration=0.5):
        """Pans the camera right"""
        self.gimbal(pitch=0, yaw=speed, duration=duration)

    def stop_camera(self):
        """Immediately stops all gimbal rotation"""
        requests.post(f"{self.base_url}/api/gimbal", params={"pitch": 0, "yaw": 0})

    def fire(self, fire_type="bead"):
        """
        Triggers robot fire:
        - "bead" (default): Real physical gel bead shot
        - "both": Real bead shot + IR laser flash & sound
        - "infrared": Simulation IR only (laser sound + LED)
        """
        requests.post(f"{self.base_url}/api/fire", params={"type": fire_type})


    def get_status(self):
        """Returns robot connection and battery status"""
        try:
            return requests.get(f"{self.base_url}/api/status", timeout=2).json()
        except Exception:
            return {"connected": False, "battery": 0}

    def get_detections(self):
        """
        Returns the list of currently detected objects from local AI vision.
        Example: [{'label': 'person', 'score': 0.92, 'bbox': [100, 50, 200, 300]}]
        """
        try:
            return requests.get(f"{self.base_url}/api/detections", timeout=1).json()
        except Exception:
            return []

    def get_snapshot(self):
        """Downloads the latest live camera snapshot (JPEG bytes)"""
        try:
            r = requests.get(f"{self.base_url}/snapshot", timeout=2)
            if r.status_code == 200:
                return r.content
        except Exception:
            pass
        return None

    def get_target(self):
        """Returns active tracking target (e.g. 'person', 'bottle', 'canette', 'all')"""
        try:
            return requests.get(f"{self.base_url}/api/target", timeout=1).json().get("target", "all")
        except Exception:
            return "all"

    def set_target(self, target="person"):
        """Sets the active tracking target on the server ('person', 'bottle', 'canette', 'all')"""
        try:
            return requests.post(f"{self.base_url}/api/target", params={"target": target}, timeout=1).json()
        except Exception:
            return {}
