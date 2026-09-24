import time
from robomaster_api import RoboMaster

def main():
    print("==================================================")
    print("    DJI ROBOMASTER S1 - AI OBJECT DETECTOR        ")
    print("==================================================")
    print("[+] Connecting to robot via local API...")
    
    robot = RoboMaster()
    status = robot.get_status()
    print(f"[+] Robot status: {status}")

    print("\n[+] Actively monitoring video stream detections...")
    print("    (Open http://localhost:8080 to see live HUD)")
    print("    Press Ctrl+C to stop.\n")

    try:
        last_count = -1
        while True:
            detections = robot.get_detections()
            count = len(detections)

            if count > 0:
                print(f"[🎯] {count} object(s) detected:")
                for d in detections:
                    label = d.get('label', 'unknown')
                    score = int(d.get('score', 0) * 100)
                    bbox = d.get('bbox', [])
                    print(f"     👉 {label.upper()} ({score}%) | Bounding box: {bbox}")
            elif last_count != 0:
                print("[...] Waiting for target objects in field of view...")

            last_count = count
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[+] Detection monitoring stopped.")

if __name__ == "__main__":
    main()
