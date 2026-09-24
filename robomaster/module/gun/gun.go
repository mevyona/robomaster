package gun

import (
	"fmt"
	"log/slog"
	"sync/atomic"
	"time"

	"github.com/brunoga/robomaster/module"
	"github.com/brunoga/robomaster/module/connection"
	"github.com/brunoga/robomaster/module/robot"
	"github.com/brunoga/robomaster/support/logger"
	"github.com/brunoga/robomaster/unitybridge"
	"github.com/brunoga/robomaster/unitybridge/unity/key"
)

// Gun is the module that controls turret firing. It supports both laser simulation
// and physical gel beads firing.
type Gun struct {
	ub     unitybridge.UnityBridge
	l      *logger.Logger
	rm     *robot.Robot
	cm     *connection.Connection
	firing atomic.Bool
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

// fireBead triggers physical gel bead firing by engaging the flywheels and feeder wheel
// for 650ms, which chambers and propels 1 bead before cleanly stopping the motors.
func (g *Gun) fireBead(times uint64) error {
	if !g.firing.CompareAndSwap(false, true) {
		return fmt.Errorf("gun is currently busy firing")
	}

	if times == 0 {
		times = 1
	}

	pulseDuration := time.Duration(times*650) * time.Millisecond
	go func() {
		time.Sleep(pulseDuration)
		_ = g.ub.DirectSendKeyValue(key.KeyRobomasterWaterGunWaterGunFire, uint64(0))
		g.firing.Store(false)
	}()

	return g.ub.DirectSendKeyValue(key.KeyRobomasterWaterGunWaterGunFire, uint64(1))
}

// fireInfrared triggers pure laser simulation (laser sound effect + red LED flash on turret)
// with a brief 120ms pulse, without chambering or propelling a bead.
func (g *Gun) fireInfrared() error {
	if !g.firing.CompareAndSwap(false, true) {
		return fmt.Errorf("gun is currently busy firing")
	}

	go func() {
		time.Sleep(120 * time.Millisecond)
		_ = g.ub.DirectSendKeyValue(key.KeyRobomasterWaterGunWaterGunFire, uint64(0))
		g.firing.Store(false)
	}()

	return g.ub.DirectSendKeyValue(key.KeyRobomasterWaterGunWaterGunFire, uint64(1))
}
