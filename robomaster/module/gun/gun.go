package gun

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/brunoga/robomaster/module"
	"github.com/brunoga/robomaster/module/connection"
	"github.com/brunoga/robomaster/module/robot"
	"github.com/brunoga/robomaster/support/logger"
	"github.com/brunoga/robomaster/unitybridge"
	"github.com/brunoga/robomaster/unitybridge/unity/key"
	"github.com/brunoga/robomaster/unitybridge/unity/result/value"
)

// Gun is the module that controls turret firing. It supports both infrared and
// beads firing.
type Gun struct {
	ub unitybridge.UnityBridge
	l  *logger.Logger
	rm *robot.Robot
	cm *connection.Connection
}

var _ module.Module = (*Gun)(nil)

// New creates a new Gun instance.
func New(ub unitybridge.UnityBridge, l *logger.Logger,
	cm *connection.Connection, rm *robot.Robot) (*Gun, error) {
	if l == nil {
		l = logger.New(slog.LevelError)
	}

	l = l.WithGroup("gun_module")

	return &Gun{
		ub: ub,
		l:  l,
		rm: rm,
		cm: cm,
	}, nil
}

// Start starts the Gun module.
func (g *Gun) Start() error {
	return g.rm.EnableFunction(robot.FunctionTypeGunControl, true)
}

// Connected returns whether the Gun module is connected.
func (g *Gun) Connected() bool {
	return g.rm.HasDevice(robot.DeviceTypeWaterGun) && g.cm.Connected()
}

// WaitForConnection waits for the Gun module to connect and returns the
// connected status.
func (g *Gun) WaitForConnection(timeout time.Duration) bool {
	start := time.Now()

	if !g.cm.WaitForConnection(timeout) {
		return false
	}

	timeout = timeout - time.Since(start)

	if !g.rm.WaitForDevices(timeout) {
		return false
	}

	return g.rm.HasDevice(robot.DeviceTypeWaterGun)
}

// Fire fires the Gun module with the given type.
func (g *Gun) Fire(typ Type) error {
	switch typ {
	case TypeBead:
		return g.fireBead(1)
	case TypeInfrared:
		return g.fireInfrared()
	}

	return fmt.Errorf("invalid gun type: %v", typ)
}

// Stop stops the Gun module.
func (g *Gun) Stop() error {
	return g.rm.EnableFunction(robot.FunctionTypeGunControl, false)
}

// String returns a string representation of the Gun module.
func (G *Gun) String() string {
	return "Gun"
}

// fireBead triggers physical gel bead firing by engaging the bead flywheels and feed mechanism.
func (g *Gun) fireBead(times uint64) error {
	if times == 0 {
		times = 1
	}

	// 1. Send high-level DJI command to fire N beads (JSON payload via PerformActionForKey)
	_ = g.ub.PerformActionForKey(key.KeyRobomasterWaterGunWaterGunFireWithTimes,
		&value.Uint64{Value: times}, nil)

	// 2. Also send DirectSendKeyValue with times to cover direct bridge handlers
	_ = g.ub.DirectSendKeyValue(key.KeyRobomasterWaterGunWaterGunFireWithTimes, times)

	// 3. Pulse KeyRobomasterWaterGunWaterGunFire (1 then 0)
	// Gel bead mechanism requires ~700ms pulse for the motors to spin up and feed a bead
	pulseDuration := time.Duration(times*700) * time.Millisecond
	go func() {
		time.Sleep(pulseDuration)
		_ = g.ub.DirectSendKeyValue(key.KeyRobomasterWaterGunWaterGunFire, uint64(0))
	}()

	return g.ub.DirectSendKeyValue(key.KeyRobomasterWaterGunWaterGunFire, uint64(1))
}

// fireInfrared triggers pure laser/infrared firing (sound and LED flash without moving the bead motors).
func (g *Gun) fireInfrared() error {
	// Send both via PerformActionForKey and DirectSendKeyValue for KeyRobomasterInfraredGunInfraredGunFire
	_ = g.ub.PerformActionForKey(key.KeyRobomasterInfraredGunInfraredGunFire,
		&value.Uint64{Value: 1}, nil)

	go func() {
		time.Sleep(150 * time.Millisecond)
		_ = g.ub.DirectSendKeyValue(key.KeyRobomasterInfraredGunInfraredGunFire, uint64(0))
	}()

	return g.ub.DirectSendKeyValue(key.KeyRobomasterInfraredGunInfraredGunFire, uint64(1))
}
