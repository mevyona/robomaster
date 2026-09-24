package main

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/brunoga/robomaster"
	"github.com/brunoga/robomaster/module/chassis"
	"github.com/brunoga/robomaster/support/logger"
)

func main() {
	robotIP := "10.156.149.194"
	fmt.Printf("[+] Initialisation de la connexion vers le RoboMaster S1 (%s)...\n", robotIP)

	l := logger.New(slog.LevelInfo)
	client, err := robomaster.NewWithTargetIP(l, robotIP)
	if err != nil {
		fmt.Printf("[!] Erreur création client: %v\n", err)
		return
	}

	fmt.Println("[+] Démarrage du client et connexion au robot...")
	err = client.Start()
	if err != nil {
		fmt.Printf("[!] Erreur démarrage: %v\n", err)
		return
	}
	defer client.Stop()

	fmt.Println("[+] Connecté avec succès au RoboMaster S1 !")

	// Test Nacelle (Gimbal)
	if client.Gimbal() != nil {
		fmt.Println("[+] Rotation nacelle vers le haut...")
		_ = client.Gimbal().SetRotationSpeed(20, 0)
		time.Sleep(1 * time.Second)
		_ = client.Gimbal().SetRotationSpeed(-20, 0)
		time.Sleep(1 * time.Second)
		_ = client.Gimbal().SetRotationSpeed(0, 0)
	}

	// Test Châssis (Mouvement)
	if client.Chassis() != nil {
		fmt.Println("[+] Avance légère du châssis (0.3 m/s pendant 1s)...")
		_ = client.Chassis().SetSpeed(chassis.ModeFPV, 0.3, 0, 0)
		time.Sleep(1 * time.Second)
		_ = client.Chassis().StopMovement(chassis.ModeFPV)

		time.Sleep(500 * time.Millisecond)

		fmt.Println("[+] Recul du châssis (-0.3 m/s pendant 1s)...")
		_ = client.Chassis().SetSpeed(chassis.ModeFPV, -0.3, 0, 0)
		time.Sleep(1 * time.Second)
		_ = client.Chassis().StopMovement(chassis.ModeFPV)
	}

	fmt.Println("[+] Test terminé avec succès !")
}
