package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"image/jpeg"
	"io"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/brunoga/robomaster"
	"github.com/brunoga/robomaster/module/camera"
	"github.com/brunoga/robomaster/module/chassis"
	"github.com/brunoga/robomaster/module/gun"
	"github.com/brunoga/robomaster/support/logger"
)

var (
	robotClient *robomaster.Client

	// Frame video cache for MJPEG streaming
	currentFrameLock sync.RWMutex
	currentJpegBytes []byte
	frameSubscribers = make(map[chan []byte]struct{})
	subscribersLock  sync.Mutex

	// Latest AI detections stored
	latestDetectionsLock sync.RWMutex
	latestDetections     = []byte("[]")

	// Selected target to track ("can", "person", "bottle", "all")
	currentTargetLock sync.RWMutex
	currentTarget     = "can"

	// Auto-fire upon target lock
	autoFireEnabledLock sync.RWMutex
	autoFireEnabled     = true

	// Sentry Standby Mode (Left/Right continuous turret sweep)
	standbyEnabledLock sync.RWMutex
	standbyEnabled     = true

	// Action log file mutex
	logFileLock sync.Mutex

	// Turret state tracking for logging
	lastGimbalLock  sync.Mutex
	lastGimbalPitch int16
	lastGimbalYaw   int16

	// Anti-runaway watchdog timer
	watchdogTimer *time.Timer
	watchdogMutex sync.Mutex
)

func getLogFilePath() string {
	if _, err := os.Stat("robomaster"); err == nil {
		return "robot_actions.log"
	}
	return "../robot_actions.log"
}

func logAction(category string, message string) {
	logFileLock.Lock()
	defer logFileLock.Unlock()

	timestamp := time.Now().Format("2006-01-02 15:04:05")
	entry := fmt.Sprintf("[%s] [%s] %s\n", timestamp, category, message)

	fmt.Print(entry)

	logPath := getLogFilePath()
	f, err := os.OpenFile(logPath, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0644)
	if err == nil {
		defer f.Close()
		_, _ = f.WriteString(entry)
	}
}

func broadcastFrame(jpegData []byte) {
	currentFrameLock.Lock()
	currentJpegBytes = jpegData
	currentFrameLock.Unlock()

	subscribersLock.Lock()
	defer subscribersLock.Unlock()
	for ch := range frameSubscribers {
		select {
		case ch <- jpegData:
		default:
		}
	}
}

func stopChassis() {
	if robotClient != nil && robotClient.Chassis() != nil {
		for i := 0; i < 3; i++ {
			_ = robotClient.Chassis().SetSpeed(chassis.ModeAngularVelocity, 0, 0, 0)
			_ = robotClient.Chassis().StopMovement(chassis.ModeAngularVelocity)
			time.Sleep(10 * time.Millisecond)
		}
	}
}

func moveChassis(x, y, z float64) {
	// Chassis wheels disabled by design: only camera and turret are active
	stopChassis()
}

func setAutoStopTimer(d time.Duration) {
	// Chassis movement disabled
}

func handleVideo(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "multipart/x-mixed-replace; boundary=frame")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "close")

	frameChan := make(chan []byte, 2)
	subscribersLock.Lock()
	frameSubscribers[frameChan] = struct{}{}
	subscribersLock.Unlock()

	defer func() {
		subscribersLock.Lock()
		delete(frameSubscribers, frameChan)
		subscribersLock.Unlock()
	}()

	notify := r.Context().Done()
	for {
		select {
		case <-notify:
			return
		case frameData := <-frameChan:
			var buf bytes.Buffer
			buf.WriteString("--frame\r\nContent-Type: image/jpeg\r\n\r\n")
			buf.Write(frameData)
			buf.WriteString("\r\n")
			_, err := w.Write(buf.Bytes())
			if err != nil {
				return
			}
			if flusher, ok := w.(http.Flusher); ok {
				flusher.Flush()
			}
		}
	}
}

func handleSnapshot(w http.ResponseWriter, r *http.Request) {
	currentFrameLock.RLock()
	data := currentJpegBytes
	currentFrameLock.RUnlock()

	if len(data) == 0 {
		http.Error(w, "No frame available yet", http.StatusServiceUnavailable)
		return
	}
	w.Header().Set("Content-Type", "image/jpeg")
	w.Write(data)
}

func handleMove(w http.ResponseWriter, r *http.Request) {
	// Wheels disabled
	stopChassis()
	w.Header().Set("Content-Type", "application/json")
	w.Write([]byte(`{"status":"disabled"}`))
}

func handleStop(w http.ResponseWriter, r *http.Request) {
	setAutoStopTimer(0)
	stopChassis()
	w.Header().Set("Content-Type", "application/json")
	w.Write([]byte(`{"status":"ok"}`))
}

func handleGimbal(w http.ResponseWriter, r *http.Request) {
	pitchStr := r.URL.Query().Get("pitch")
	yawStr := r.URL.Query().Get("yaw")

	pitch, _ := strconv.ParseInt(pitchStr, 10, 16)
	yaw, _ := strconv.ParseInt(yawStr, 10, 16)

	p := int16(pitch)
	y := int16(yaw)

	lastGimbalLock.Lock()
	if p != lastGimbalPitch || y != lastGimbalYaw {
		lastGimbalPitch = p
		lastGimbalYaw = y
		if p == 0 && y == 0 {
			logAction("TURRET", "Turret rotation stopped")
		} else {
			logAction("TURRET", fmt.Sprintf("Turret movement: pitch=%d yaw=%d", p, y))
		}
	}
	lastGimbalLock.Unlock()

	if robotClient != nil && robotClient.Gimbal() != nil {
		if p == 0 && y == 0 {
			_ = robotClient.Gimbal().StopRotation()
		} else {
			_ = robotClient.Gimbal().SetRotationSpeed(p, y)
		}
	}
	w.Header().Set("Content-Type", "application/json")
	w.Write([]byte(`{"status":"ok"}`))
}

func handleFire(w http.ResponseWriter, r *http.Request) {
	if robotClient != nil && robotClient.Gun() != nil {
		err := robotClient.Gun().Fire(gun.TypeInfrared)
		if err != nil {
			logAction("ERROR", fmt.Sprintf("Infrared fire error: %v", err))
		} else {
			logAction("FIRE", "Infrared fire triggered")
		}
	}
	w.Header().Set("Content-Type", "application/json")
	w.Write([]byte(`{"status":"ok"}`))
}

func handleAutoFire(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodPost {
		e := r.URL.Query().Get("enabled")
		autoFireEnabledLock.Lock()
		autoFireEnabled = (e == "1" || e == "true" || e == "on")
		autoFireEnabledLock.Unlock()
		logAction("CONFIG", fmt.Sprintf("Auto-fire on lock setting updated: %v", autoFireEnabled))
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(`{"status":"ok"}`))
		return
	}
	autoFireEnabledLock.RLock()
	af := autoFireEnabled
	autoFireEnabledLock.RUnlock()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]bool{"auto_fire": af})
}

func handleStandby(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodPost {
		e := r.URL.Query().Get("enabled")
		standbyEnabledLock.Lock()
		standbyEnabled = (e == "1" || e == "true" || e == "on")
		standbyEnabledLock.Unlock()
		logAction("CONFIG", fmt.Sprintf("Sentry standby mode updated: %v", standbyEnabled))
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(`{"status":"ok"}`))
		return
	}
	standbyEnabledLock.RLock()
	st := standbyEnabled
	standbyEnabledLock.RUnlock()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]bool{"standby": st})
}

func handleStatus(w http.ResponseWriter, r *http.Request) {
	battery := uint8(0)
	if robotClient != nil && robotClient.Robot() != nil {
		battery = robotClient.Robot().BatteryPowerPercent()
	}
	res := map[string]interface{}{
		"connected": robotClient != nil,
		"battery":   battery,
	}
	if robotClient != nil && robotClient.Gimbal() != nil {
		att := robotClient.Gimbal().Attitude()
		if att != nil {
			res["yaw"] = att.Yaw
			res["pitch"] = att.Pitch
			res["yaw_speed"] = att.YawSpeed
			res["pitch_speed"] = att.PitchSpeed
		}
	}
	data, _ := json.Marshal(res)
	w.Header().Set("Content-Type", "application/json")
	w.Write(data)
}

func handleDetections(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodPost {
		data, err := io.ReadAll(r.Body)
		if err == nil && len(data) > 0 {
			latestDetectionsLock.Lock()
			latestDetections = data
			latestDetectionsLock.Unlock()
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(`{"status":"ok"}`))
		return
	}
	latestDetectionsLock.RLock()
	data := latestDetections
	latestDetectionsLock.RUnlock()
	w.Header().Set("Content-Type", "application/json")
	w.Write(data)
}

func handleTarget(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodPost {
		t := r.URL.Query().Get("target")
		if t == "" {
			var body struct {
				Target string `json:"target"`
			}
			if err := json.NewDecoder(r.Body).Decode(&body); err == nil && body.Target != "" {
				t = body.Target
			}
		}
		if t != "" {
			currentTargetLock.Lock()
			currentTarget = strings.ToLower(strings.TrimSpace(t))
			currentTargetLock.Unlock()
			logAction("CONFIG", fmt.Sprintf("Tracking target updated: %s", strings.ToUpper(currentTarget)))
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(`{"status":"ok"}`))
		return
	}

	currentTargetLock.RLock()
	t := currentTarget
	currentTargetLock.RUnlock()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"target": t})
}

func handleLog(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodPost {
		cat := r.URL.Query().Get("cat")
		msg := r.URL.Query().Get("msg")
		if cat == "" {
			cat = "ACTION"
		}
		if msg != "" {
			logAction(cat, msg)
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(`{"status":"ok"}`))
		return
	}

	// GET : affichage du fichier log
	logPath := getLogFilePath()
	data, err := os.ReadFile(logPath)
	if err != nil {
		w.Header().Set("Content-Type", "text/plain; charset=utf-8")
		w.Write([]byte("No actions recorded yet."))
		return
	}
	w.Header().Set("Content-Type", "text/plain; charset=utf-8")
	w.Write(data)
}

func handleDashboard(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	w.Write([]byte(dashboardHTML))
}

func main() {
	robotIP := "10.156.149.194"
	if len(os.Args) > 1 {
		robotIP = os.Args[1]
	}

	fmt.Printf("====================================================\n")
	fmt.Printf("    DJI ROBOMASTER S1 - PC CONTROL SERVER      \n")
	fmt.Printf("====================================================\n")
	fmt.Printf("[+] Connecting to robot at %s...\n", robotIP)

	l := logger.New(slog.LevelWarn)
	var err error
	robotClient, err = robomaster.NewWithTargetIP(l, robotIP)
	if err != nil {
		fmt.Printf("[!] Error creating client: %v\n", err)
		return
	}

	err = robotClient.Start()
	if err != nil {
		fmt.Printf("[!] Connection error: %v\n", err)
		logAction("ERREUR", fmt.Sprintf("Failed to connect to RoboMaster S1 (%s): %v", robotIP, err))
		return
	}
	defer func() {
		logAction("DISCONNECTION", "Server stopped and robot disconnected")
		stopChassis()
		robotClient.Stop()
	}()

	// Safety lock at startup
	stopChassis()

	fmt.Println("[+] Connected to RoboMaster S1!")
	batt := uint8(0)
	if robotClient.Robot() != nil {
		batt = robotClient.Robot().BatteryPowerPercent()
	}
	logAction("CONNECTION", fmt.Sprintf("Connected successfully to RoboMaster S1 (%s) - Battery: %d%%", robotIP, batt))

	// Camera initialization
	if robotClient.Camera() != nil {
		fmt.Println("[+] Starting camera video stream...")
		_, err := robotClient.Camera().AddVideoCallback(func(frame *camera.RGB) {
			if frame == nil {
				return
			}
			var buf bytes.Buffer
			err := jpeg.Encode(&buf, frame, &jpeg.Options{Quality: 70})
			if err == nil {
				broadcastFrame(buf.Bytes())
			}
		})
		if err != nil {
			fmt.Printf("[!] Camera error: %v\n", err)
		} else {
			fmt.Println("[+] Camera stream active!")
		}
	}

	// Routes HTTP
	http.HandleFunc("/", handleDashboard)
	http.HandleFunc("/video", handleVideo)
	http.HandleFunc("/snapshot", handleSnapshot)
	http.HandleFunc("/api/move", handleMove)
	http.HandleFunc("/api/stop", handleStop)
	http.HandleFunc("/api/gimbal", handleGimbal)
	http.HandleFunc("/api/fire", handleFire)
	http.HandleFunc("/api/status", handleStatus)
	http.HandleFunc("/api/detections", handleDetections)
	http.HandleFunc("/api/target", handleTarget)
	http.HandleFunc("/api/autofire", handleAutoFire)
	http.HandleFunc("/api/standby", handleStandby)
	http.HandleFunc("/api/log", handleLog)
	http.HandleFunc("/api/logs", handleLog)

	serverPort := "8080"
	fmt.Printf("\n[🚀] RoboMaster Camera Cockpit available at:\n")
	fmt.Printf("     👉 http://localhost:%s\n\n", serverPort)
	fmt.Println("Controls (browser):")
	fmt.Println("  - Keyboard Arrows : Up / Down / Left / Right (Turret orientation)")
	fmt.Println("  - Spacebar        : Infrared fire")
	fmt.Println("  - Chassis Wheels  : DISABLED (Total indoor safety)")
	fmt.Println("\nPress Ctrl+C to stop.")

	go func() {
		if err := http.ListenAndServe(":"+serverPort, nil); err != nil {
			fmt.Printf("[!] HTTP server error: %v\n", err)
		}
	}()

	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)
	<-sigChan

	fmt.Println("\nStopping server and disconnecting...")
	stopChassis()
}

const dashboardHTML = `<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8">
  <title>RoboMaster S1 - Camera Cockpit</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; user-select: none; }
    body {
      background: #0d1117;
      color: #c9d1d9;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      height: 100vh;
      overflow: hidden;
    }
    header {
      width: 100%;
      padding: 12px 24px;
      background: #161b22;
      border-bottom: 1px solid #30363d;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .title {
      font-size: 1.25rem;
      font-weight: 700;
      color: #58a6ff;
      display: flex;
      align-items: center;
      gap: 10px;
    }
    .badge {
      font-size: 0.75rem;
      background: #238636;
      color: white;
      padding: 3px 8px;
      border-radius: 12px;
    }
    .badge-ai {
      font-size: 0.75rem;
      background: #1f6feb;
      color: white;
      padding: 3px 8px;
      border-radius: 12px;
      margin-left: 8px;
    }
    .main-view {
      flex: 1;
      display: flex;
      width: 100%;
      max-width: 1440px;
      gap: 20px;
      padding: 20px;
      overflow: hidden;
    }
    .video-container {
      flex: 2;
      background: #000;
      border-radius: 12px;
      overflow: hidden;
      border: 1px solid #30363d;
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 0 8px 24px rgba(0,0,0,0.5);
    }
    .video-container img {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
    }
    #overlayCanvas {
      position: absolute;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      pointer-events: none;
    }
    .hud-info {
      position: absolute;
      top: 12px;
      left: 16px;
      background: rgba(13, 17, 23, 0.8);
      backdrop-filter: blur(4px);
      padding: 6px 12px;
      border-radius: 6px;
      border: 1px solid rgba(88, 166, 255, 0.3);
      font-family: monospace;
      font-size: 0.8rem;
      color: #58a6ff;
      pointer-events: none;
    }
    .side-panel {
      flex: 1;
      display: flex;
      flex-direction: column;
      gap: 14px;
      overflow-y: auto;
      max-height: calc(100vh - 90px);
      padding-right: 4px;
    }
    .card {
      background: #161b22;
      border: 1px solid #30363d;
      border-radius: 12px;
      padding: 14px 16px;
    }
    .card h3 {
      font-size: 0.85rem;
      color: #8b949e;
      text-transform: uppercase;
      margin-bottom: 10px;
      letter-spacing: 0.5px;
    }
    .stat-row {
      display: flex;
      justify-content: space-between;
      margin-bottom: 6px;
      font-size: 0.88rem;
    }
    .controls-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 6px;
    }
    .btn {
      background: #21262d;
      color: #f0f6fc;
      border: 1px solid #30363d;
      border-radius: 6px;
      padding: 10px 6px;
      font-size: 0.85rem;
      font-weight: 600;
      cursor: pointer;
      text-align: center;
      transition: all 0.1s ease;
    }
    .btn:hover, .btn.active {
      background: #388bfd;
      border-color: #58a6ff;
      color: white;
    }
    .btn-fire {
      background: #da3633;
      border-color: #f85149;
      color: white;
      grid-column: span 3;
      padding: 12px;
      font-size: 1rem;
      margin-top: 6px;
    }
    .btn-fire:hover, .btn-fire.active {
      background: #f85149;
    }
    .detections-box {
      background: #0d1117;
      border: 1px solid #30363d;
      border-radius: 6px;
      padding: 8px;
      min-height: 50px;
      max-height: 120px;
      overflow-y: auto;
      font-family: monospace;
      font-size: 0.8rem;
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .det-item {
      display: flex;
      justify-content: space-between;
      color: #7ee787;
      background: rgba(46, 160, 67, 0.15);
      padding: 3px 6px;
      border-radius: 4px;
    }
    .instructions {
      font-size: 0.78rem;
      color: #8b949e;
      line-height: 1.5;
    }
    kbd {
      background: #21262d;
      border: 1px solid #30363d;
      border-radius: 4px;
      padding: 1px 5px;
      font-size: 0.7rem;
      color: #f0f6fc;
      font-family: monospace;
    }
  </style>
</head>
<body>
  <header>
    <div class="title">
      🤖 DJI RoboMaster S1 Cockpit
      <span class="badge" id="statusBadge">CONNECTED</span>
      <span class="badge-ai" id="aiBadge">LOCAL PC AI READY</span>
      <a href="/api/logs" target="_blank" style="font-size: 0.75rem; color: #58a6ff; text-decoration: none; margin-left: 10px; border: 1px solid #388bfd; padding: 2px 8px; border-radius: 4px;">📜 View Logs</a>
    </div>
    <div style="font-size: 0.85rem; color: #8b949e;">IP: 10.156.149.194</div>
  </header>

  <div class="main-view">
    <div class="video-container" id="videoWrapper">
      <img src="/video" alt="RoboMaster Camera Stream" id="videoFeed" crossorigin="anonymous">
      <canvas id="overlayCanvas"></canvas>
      <div class="hud-info" id="hudInfo">100% SMOOTH HD VIDEO STREAM</div>
    </div>

    <div class="side-panel">
      <!-- État général -->
      <div class="card">
        <h3>Robot Status</h3>
        <div class="stat-row">
          <span>Battery</span>
          <strong id="battVal">-- %</strong>
        </div>
        <div class="stat-row">
          <span>Turret Yaw</span>
          <strong id="yawVal" style="color: #58a6ff;">--°</strong>
        </div>
        <div class="stat-row">
          <span>Mode</span>
          <strong style="color: #58a6ff;">Pure Camera & Turret</strong>
        </div>
        <div class="stat-row">
          <span>Chassis Wheels</span>
          <strong style="color: #8b949e;">Locked (Safety)</strong>
        </div>
      </div>

      <!-- Cible de Repérage & Suivi IA -->
      <div class="card">
        <h3>🎯 Target to Track</h3>
        <div style="font-size: 0.8rem; color: #8b949e; margin-bottom: 8px;">
          Choose object for the AI to track and follow:
        </div>
        <select id="targetSelect" onchange="onTargetChange()" style="width: 100%; padding: 8px 10px; background: #21262d; color: #f0f6fc; border: 1px solid #30363d; border-radius: 6px; font-size: 0.88rem; font-weight: 500; cursor: pointer; outline: none;">
          <option value="can" selected>🥫 Soda can / Cup</option>
          <option value="person">👤 Person (closest only)</option>
          <option value="bottle">🍾 Bottle</option>
          <option value="all">🎯 All 3 targets</option>
        </select>
        <div id="activeTargetBadge" style="margin-top: 8px; font-size: 0.8rem; color: #58a6ff; font-family: monospace;">
          Active Target: <strong>SODA CAN</strong>
        </div>
        <div style="margin-top: 10px; display: flex; align-items: center; justify-content: space-between; padding: 8px 10px; background: rgba(218, 54, 51, 0.12); border: 1px solid rgba(248, 81, 73, 0.35); border-radius: 6px;">
          <div>
            <div style="font-size: 0.85rem; font-weight: 600; color: #f85149;">💥 Auto-Fire on Lock</div>
            <div style="font-size: 0.72rem; color: #8b949e;">Fires once when target is centered</div>
          </div>
          <input type="checkbox" id="autoFireToggle" onchange="toggleAutoFire(this.checked)" checked style="width: 18px; height: 18px; cursor: pointer; accent-color: #f85149;">
        </div>
        <div style="margin-top: 8px; display: flex; align-items: center; justify-content: space-between; padding: 8px 10px; background: rgba(56, 139, 253, 0.12); border: 1px solid rgba(56, 139, 253, 0.35); border-radius: 6px;">
          <div>
            <div style="font-size: 0.85rem; font-weight: 600; color: #58a6ff;">📡 Sentry Standby Mode</div>
            <div style="font-size: 0.72rem; color: #8b949e;">Sweeps left/right until target detected</div>
          </div>
          <input type="checkbox" id="standbyToggle" onchange="toggleStandby(this.checked)" checked style="width: 18px; height: 18px; cursor: pointer; accent-color: #388bfd;">
        </div>
      </div>

      <!-- Reconnaissance & Détection IA (PC) -->
      <div class="card">
        <h3>Local AI Vision (on PC)</h3>
        <div style="font-size: 0.8rem; color: #8b949e; margin-bottom: 8px;">
          AI (YOLOv8 / OpenCV) runs locally on PC CPU with 0% browser lag.
        </div>
        <div style="font-size: 0.8rem; color: #8b949e; margin-bottom: 4px;">Objects detected by PC:</div>
        <div class="detections-box" id="detectionsList">
          <span style="color: #8b949e;">Waiting for detections (run start_all.bat)...</span>
        </div>
      </div>

      <!-- Contrôle Caméra / Tourelle -->
      <div class="card">
        <h3>Turret Orientation</h3>
        <div class="controls-grid">
          <div></div>
          <button class="btn" id="btnUp" onmousedown="startContinuousGimbal(30, 0)" onmouseup="stopGimbal()" onmouseleave="stopGimbal()">▲ Up<br><kbd>↑</kbd></button>
          <div></div>
          <button class="btn" id="btnLeft" onmousedown="startContinuousGimbal(0, -40)" onmouseup="stopGimbal()" onmouseleave="stopGimbal()">◄ Left<br><kbd>←</kbd></button>
          <button class="btn" onclick="stopGimbal()">⏹ Stop</button>
          <button class="btn" id="btnRight" onmousedown="startContinuousGimbal(0, 40)" onmouseup="stopGimbal()" onmouseleave="stopGimbal()">► Right<br><kbd>→</kbd></button>
          <div></div>
          <button class="btn" id="btnDown" onmousedown="startContinuousGimbal(-30, 0)" onmouseup="stopGimbal()" onmouseleave="stopGimbal()">▼ Down<br><kbd>↓</kbd></button>
          <div></div>
        </div>
        <div style="margin-top: 8px;">
          <button class="btn btn-fire" id="btnFire" onclick="fire()" style="width: 100%;">💥 INFRARED FIRE (Space)</button>
        </div>
      </div>

      <!-- Instructions -->
      <div class="card instructions">
        <strong>Commandes :</strong><br>
        • <strong>Keyboard Arrows:</strong> <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> to orient the turret.<br>
        • <strong>Fire:</strong> <kbd>Space</kbd> key.<br>
        • <strong>Local PC AI:</strong> Run <kbd>start_all.bat</kbd> for automatic tracking with zero lag.
      </div>
    </div>
  </div>

  <script>
    const video = document.getElementById('videoFeed');
    const canvas = document.getElementById('overlayCanvas');
    const ctx = canvas.getContext('2d');
    const detectionsListEl = document.getElementById('detectionsList');
    const hudInfo = document.getElementById('hudInfo');
    const aiBadge = document.getElementById('aiBadge');

    let activeKeys = {};
    let gimbalInterval = null;
    let latestDetections = [];

    // Récupération des détections envoyées par l'IA locale (toutes les 100ms)
    async function fetchDetections() {
      try {
        const res = await fetch('/api/detections');
        if (res.ok) {
          latestDetections = await res.json();
          updateDOMList(latestDetections);
        }
      } catch (e) {}
    }
    setInterval(fetchDetections, 100);

    function updateDOMList(detections) {
      const isStandby = document.getElementById('standbyToggle')?.checked;
      if (!detections || detections.length === 0) {
        detectionsListEl.innerHTML = '<span style="color: #8b949e;">No target detected</span>';
        if (isStandby) {
          hudInfo.innerText = '📡 SENTRY STANDBY: TURRET PATROL (SWEEP)';
          hudInfo.style.color = '#58a6ff';
          hudInfo.style.borderColor = '#388bfd';
        } else {
          hudInfo.innerText = 'VISION: NO TARGET DETECTED (IDLE)';
          hudInfo.style.color = '#8b949e';
          hudInfo.style.borderColor = 'rgba(88, 166, 255, 0.3)';
        }
        return;
      }
      aiBadge.innerText = 'LOCAL AI: ' + detections.length + ' DETECTION(S)';
      aiBadge.style.background = '#238636';

      let html = '';
      let targetCount = 0;
      let isAnyLocked = false;
      let isAnyFired = false;

      detections.forEach((det) => {
        const label = det.label || det.class || 'objet';
        const score = Math.round((det.score || 0) * 100);
        const isTarget = det.is_target !== false;
        const isLocked = det.is_locked === true;
        const justFired = det.just_fired === true;
        const isHit = det.is_hit === true;

        if (isTarget && !isHit) targetCount++;
        if (isLocked) isAnyLocked = true;
        if (justFired) isAnyFired = true;

        let icon = isHit ? '💥' : (isTarget ? (isLocked ? '🔒' : '🎯') : '👁️');
        let badge = isHit
          ? '<span style="color:#d97706;font-weight:bold;border:1px solid #d97706;padding:1px 4px;border-radius:3px;">HIT (ELIMINATED)</span>'
          : (isTarget ? (isLocked ? '<span style="color:#f85149;font-weight:bold;">LOCK (TORSO)</span>' : '<span style="color:#7ee787;">TARGET</span>') : '<span style="color:#8b949e;">other</span>');
        if (det.aiming_at_face) {
          badge += ' <span style="color:#ff9800;font-size:0.7rem;border:1px solid #d97706;padding:1px 4px;border-radius:3px;">PROTECT FACE</span>';
        }

        html += ` + "`" + `<div class="det-item">
          <span>${icon} <strong>${label.toUpperCase()}</strong> (${score}%)</span>
          ${badge}
        </div>` + "`" + `;
      });

      const isAnyAimingFace = detections.some(d => d.aiming_at_face && d.is_target);

      if (isAnyFired) {
        hudInfo.innerText = '💥💥 TARGET HIT! SWITCHING TO NEXT TARGET... 💥💥';
        hudInfo.style.color = '#ff7b72';
        hudInfo.style.borderColor = '#f85149';
      } else if (isAnyAimingFace) {
        hudInfo.innerText = '⚠️ FACE PROTECTION ACTIVE: RETARGETING TO TORSO (FIRE INHIBITED)';
        hudInfo.style.color = '#ff9800';
        hudInfo.style.borderColor = '#d97706';
      } else if (isAnyLocked) {
        hudInfo.innerText = '🔒 TARGET LOCKED ON TORSO AT CENTER (FACE CLEAR)';
        hudInfo.style.color = '#f85149';
        hudInfo.style.borderColor = '#f85149';
      } else {
        hudInfo.innerText = 'VISION: ' + detections.length + ' OBJECT(S) DETECTED';
        hudInfo.style.color = '#58a6ff';
        hudInfo.style.borderColor = 'rgba(88, 166, 255, 0.3)';
      }

      detectionsListEl.innerHTML = html;
    }

    // Gestion du choix de la cible IA
    const TARGET_LABELS = {
      'person': 'PERSON (CLOSEST)',
      'bottle': 'BOTTLE',
      'canette': 'SODA CAN', 'can': 'SODA CAN',
      'all': 'ALL 3 TARGETS'
    };

    async function onTargetChange() {
      const sel = document.getElementById('targetSelect');
      setTarget(sel.value);
    }

    async function setTarget(target) {
      try {
        await fetch('/api/target?target=' + encodeURIComponent(target), { method: 'POST' });
        const label = TARGET_LABELS[target] || target.toUpperCase();
        document.getElementById('activeTargetBadge').innerHTML = 'Active Target: <strong>' + label + '</strong>';
      } catch (e) {}
    }

    async function toggleAutoFire(enabled) {
      try {
        await fetch('/api/autofire?enabled=' + (enabled ? 'true' : 'false'), { method: 'POST' });
      } catch (e) {}
    }

    async function fetchAutoFire() {
      try {
        const res = await fetch('/api/autofire');
        if (res.ok) {
          const data = await res.json();
          if (typeof data.auto_fire === 'boolean') {
            document.getElementById('autoFireToggle').checked = data.auto_fire;
          }
        }
      } catch (e) {}
    }
    fetchAutoFire();

    async function toggleStandby(enabled) {
      try {
        await fetch('/api/standby?enabled=' + (enabled ? 'true' : 'false'), { method: 'POST' });
      } catch (e) {}
    }

    async function fetchStandby() {
      try {
        const res = await fetch('/api/standby');
        if (res.ok) {
          const data = await res.json();
          if (typeof data.standby === 'boolean') {
            document.getElementById('standbyToggle').checked = data.standby;
          }
        }
      } catch (e) {}
    }
    fetchStandby();

    async function fetchCurrentTarget() {
      try {
        const res = await fetch('/api/target');
        if (res.ok) {
          const data = await res.json();
          if (data.target) {
            const label = TARGET_LABELS[data.target] || data.target.toUpperCase();
            document.getElementById('activeTargetBadge').innerHTML = 'Active Target: <strong>' + label + '</strong>';
            const sel = document.getElementById('targetSelect');
            for (let i = 0; i < sel.options.length; i++) {
              if (sel.options[i].value === data.target) {
                sel.selectedIndex = i;
                break;
              }
            }
          }
        }
      } catch (e) {}
    }
    fetchCurrentTarget();

    // Boucle de rendu graphique du HUD (60 FPS, zéro calcul CPU, fluide à 100%)
    function renderHUD() {
      if (video.videoWidth || video.naturalWidth) {
        const vw = video.naturalWidth || video.videoWidth || video.clientWidth;
        const vh = video.naturalHeight || video.videoHeight || video.clientHeight;
        if (canvas.width !== vw || canvas.height !== vh) {
          canvas.width = vw;
          canvas.height = vh;
        }

        ctx.clearRect(0, 0, canvas.width, canvas.height);
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;

        let hasLock = false;
        let hasFired = false;

        // Vérifier si un objet est verrouillé
        if (latestDetections && latestDetections.length > 0) {
          latestDetections.forEach(d => {
            if (d.is_locked) hasLock = true;
            if (d.just_fired) hasFired = true;
          });
        }

        // Réticule central
        ctx.strokeStyle = hasLock ? '#f85149' : 'rgba(0, 255, 136, 0.45)';
        ctx.lineWidth = hasLock ? 2 : 1;
        ctx.beginPath();
        ctx.arc(centerX, centerY, hasLock ? 24 : 20, 0, 2 * Math.PI);
        ctx.stroke();

        if (hasLock) {
          ctx.beginPath();
          ctx.arc(centerX, centerY, 6, 0, 2 * Math.PI);
          ctx.fillStyle = 'rgba(248, 81, 73, 0.8)';
          ctx.fill();
        }

        // Dessin des boîtes de détection envoyées par le PC
        if (latestDetections && latestDetections.length > 0) {
          latestDetections.forEach((det) => {
            const bbox = det.bbox || [];
            if (bbox.length === 4) {
              const [x, y, w, h] = bbox;
              const isTarget = det.is_target !== false;
              const isLocked = det.is_locked === true;
              const justFired = det.just_fired === true;
              const isHit = det.is_hit === true;

              let strokeColor = '#7ee787';
              let fillColor = 'rgba(35, 134, 54, 0.85)';
              let labelPrefix = '🎯 ';

              if (isHit) {
                strokeColor = '#d97706';
                fillColor = 'rgba(217, 119, 6, 0.85)';
                labelPrefix = '💥 HIT : ';
              } else if (!isTarget) {
                strokeColor = 'rgba(88, 166, 255, 0.65)';
                fillColor = 'rgba(22, 27, 34, 0.85)';
                labelPrefix = '👁️ ';
              } else if (isLocked) {
                strokeColor = '#f85149';
                fillColor = 'rgba(218, 54, 51, 0.9)';
                labelPrefix = '🔒 LOCK : ';
              }

              ctx.strokeStyle = strokeColor;
              ctx.lineWidth = isLocked ? 3 : 2;
              if (isHit) ctx.setLineDash([5, 3]);
              ctx.strokeRect(x, y, w, h);
              if (isHit) ctx.setLineDash([]);

              // Coins tactiques
              const cLen = Math.min(16, w / 4, h / 4);
              ctx.lineWidth = isLocked ? 4 : 3;
              ctx.beginPath();
              ctx.moveTo(x, y + cLen); ctx.lineTo(x, y); ctx.lineTo(x + cLen, y);
              ctx.moveTo(x + w - cLen, y); ctx.lineTo(x + w, y); ctx.lineTo(x + w, y + cLen);
              ctx.moveTo(x, y + h - cLen); ctx.lineTo(x, y + h); ctx.lineTo(x + cLen, y + h);
              ctx.moveTo(x + w - cLen, y + h); ctx.lineTo(x + w, y + h); ctx.lineTo(x + w, y + h - cLen);
              ctx.stroke();

              // Zone d'exclusion visage (Face Protection)
              if (det.face_box && det.face_box.length === 4) {
                const [fx, fy, fw, fh] = det.face_box;
                ctx.save();
                ctx.strokeStyle = '#ff9800';
                ctx.lineWidth = 2;
                ctx.setLineDash([4, 3]);
                ctx.strokeRect(fx, fy, fw, fh);
                ctx.fillStyle = 'rgba(217, 119, 6, 0.9)';
                ctx.font = 'bold 10px monospace';
                ctx.fillRect(fx, Math.max(0, fy - 16), 115, 15);
                ctx.fillStyle = '#ffffff';
                ctx.fillText('🚫 NO-FIRE: FACE', fx + 3, Math.max(11, fy - 5));
                ctx.restore();
              }

              // Réticule point cible sécurisé (Torse)
              let aimX = x + w / 2;
              let aimY = y + h / 2;
              if (det.aim_point && det.aim_point.length === 2) {
                aimX = det.aim_point[0];
                aimY = det.aim_point[1];
                ctx.save();
                ctx.strokeStyle = isLocked ? '#f85149' : '#388bfd';
                ctx.fillStyle = isLocked ? '#f85149' : '#388bfd';
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.arc(aimX, aimY, 7, 0, 2 * Math.PI);
                ctx.stroke();
                ctx.beginPath();
                ctx.arc(aimX, aimY, 2.5, 0, 2 * Math.PI);
                ctx.fill();
                ctx.font = 'bold 10px monospace';
                ctx.fillText('🎯 TORSO', aimX + 10, aimY + 4);
                ctx.restore();
              }

              // Ligne de visée rouge en pointillés vers le centre quand verrouillé
              if (isLocked) {
                ctx.strokeStyle = 'rgba(248, 81, 73, 0.75)';
                ctx.lineWidth = 1.5;
                ctx.setLineDash([4, 4]);
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.lineTo(aimX, aimY);
                ctx.stroke();
                ctx.setLineDash([]);
              }

              // Étiquette
              const label = det.label || det.class || 'objet';
              const score = Math.round((det.score || 0) * 100);
              const labelText = labelPrefix + label.toUpperCase() + ' ' + score + '%';
              ctx.font = 'bold 12px monospace';
              const txtWidth = ctx.measureText(labelText).width;
              ctx.fillStyle = fillColor;
              ctx.fillRect(x, Math.max(0, y - 20), txtWidth + 10, 18);
              ctx.fillStyle = '#ffffff';
              ctx.fillText(labelText, x + 5, Math.max(13, y - 6));
            }
          });
        }

        // Bannière visuelle lors d'un tir
        if (hasFired) {
          ctx.fillStyle = 'rgba(218, 54, 51, 0.4)';
          ctx.fillRect(0, centerY - 35, canvas.width, 70);
          ctx.font = 'bold 24px monospace';
          ctx.fillStyle = '#ffffff';
          ctx.textAlign = 'center';
          ctx.fillText('💥💥 TARGET HIT! SWITCHING TO NEXT TARGET... 💥💥', centerX, centerY + 8);
          ctx.textAlign = 'left';
        }
      }
      requestAnimationFrame(renderHUD);
    }
    requestAnimationFrame(renderHUD);

    // Commandes Gimbal & Tourelle
    function sendGimbal(pitch, yaw) {
      fetch('/api/gimbal?pitch=' + pitch + '&yaw=' + yaw, { method: 'POST' });
    }

    function startContinuousGimbal(pitch, yaw) {
      stopGimbal();
      sendGimbal(pitch, yaw);
      gimbalInterval = setInterval(() => sendGimbal(pitch, yaw), 150);
    }

    function stopGimbal() {
      if (gimbalInterval) {
        clearInterval(gimbalInterval);
        gimbalInterval = null;
      }
      sendGimbal(0, 0);
      ['btnUp', 'btnDown', 'btnLeft', 'btnRight'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.classList.remove('active');
      });
    }

    function fire() {
      fetch('/api/fire', { method: 'POST' });
      const f = document.getElementById('btnFire');
      f.classList.add('active');
      setTimeout(() => f.classList.remove('active'), 200);
    }

    // Gestion du clavier (Orientation Tourelle)
    window.addEventListener('keydown', (e) => {
      const code = e.code;
      if (activeKeys[code]) return;
      activeKeys[code] = true;

      if (code === 'ArrowUp') {
        startContinuousGimbal(30, 0);
        const b = document.getElementById('btnUp');
        if (b) b.classList.add('active');
      } else if (code === 'ArrowDown') {
        startContinuousGimbal(-30, 0);
        const b = document.getElementById('btnDown');
        if (b) b.classList.add('active');
      } else if (code === 'ArrowLeft') {
        startContinuousGimbal(0, -40);
        const b = document.getElementById('btnLeft');
        if (b) b.classList.add('active');
      } else if (code === 'ArrowRight') {
        startContinuousGimbal(0, 40);
        const b = document.getElementById('btnRight');
        if (b) b.classList.add('active');
      } else if (code === 'Space') {
        fire();
      }
    });

    window.addEventListener('keyup', (e) => {
      const code = e.code;
      delete activeKeys[code];

      if (['arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(code.toLowerCase())) {
        stopGimbal();
      }
    });

    window.addEventListener('blur', () => {
      activeKeys = {};
      stopGimbal();
    });

    // Live battery and turret telemetry update
    setInterval(() => {
      fetch('/api/status').then(r => r.json()).then(data => {
        if (data.battery > 0) {
          document.getElementById('battVal').innerText = data.battery + ' %';
        }
        if (typeof data.yaw === 'number') {
          const yEl = document.getElementById('yawVal');
          if (yEl) {
            yEl.innerText = (data.yaw > 0 ? '+' : '') + data.yaw.toFixed(1) + '°';
            if (data.yaw >= 235 || data.yaw <= -235) {
              yEl.style.color = '#f85149';
            } else {
              yEl.style.color = '#7ee787';
            }
          }
        }
      }).catch(() => {});
    }, 500);
  </script>
</body>
</html>`
