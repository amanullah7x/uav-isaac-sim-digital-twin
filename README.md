# Autonomous UAV Digital Twin (PX4 + NVIDIA Isaac Sim + ROS2 Humble)

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Simulation](https://img.shields.io/badge/Simulator-NVIDIA%20Isaac%20Sim%204.0%2B-green.svg)]()
[![Autopilot](https://img.shields.io/badge/Autopilot-PX4%20Autopilot%20SITL-blue.svg)]()
[![Middleware](https://img.shields.io/badge/Middleware-ROS2%20Humble-purple.svg)]()

> A high-fidelity software-in-the-loop (SITL) digital twin platform coupling **NVIDIA Isaac Sim** with **PX4 autopilot firmware** and offboard **ROS2 Humble** visual servoing nodes. Validates dynamic target-following logic, aerodynamic response, and vision pipelines in physics-accurate simulation with zero physical crash risk.

---

## 📽️ Demo & Visual Output

Split-screen simulation-in-the-loop recording showing the **Kamikaze FPV Strike Controller** on the left and **NVIDIA Isaac Sim** quadcopter physics simulation on the right:

![Isaac Sim SITL Demo](assets/demo.gif)

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ NVIDIA Isaac Sim 4.0.1 (RTX Physics, Aerodynamics, Sensors) │
└──────────────┬───────────────────────────────▲──────────────┘
               │ (Synthetic 30 FPS Camera)     │ (Offboard Setpoint)
               ▼                               │
┌──────────────────────────────┐ ┌─────────────┴──────────────┐
│ Perception Node (ROS2 YOLO)  │ │ PX4 Autopilot SITL         │
│ - Target BBox Localization   │ │ - Full flight dynamics     │
└──────────────┬───────────────┘ └─────────────▲──────────────┘
               │                               │
               ▼ (Target Offset Error)         │ (MAVLink Offboard)
┌──────────────────────────────────────────────┴──────────────┐
│ Visual Servoing PID Controller                              │
│ - Body-frame velocity vector (Vx, Vy, Vz, YawRate)          │
└─────────────────────────────────────────────────────────────┘
```

---

## ⚙️ Key Technical Challenges & Solutions

### 1. High-Fidelity Pre-Flight Validation Without Hardware Risk
* **Problem:** Testing visual servoing and aggressive flight control maneuvers on physical prototype drones carries high crash risks and damages expensive companion computers.
* **Solution:** Replicated the quadcopter aerodynamics, camera sensor intrinsics/extrinsics, and lighting environments inside NVIDIA Isaac Sim. All perception algorithms run against synthetic RTX camera streams, closing the loop with PX4 SITL over bi-directional MAVLink bridges.

### 2. Synchronization & Telemetry Lag
* **Problem:** Time synchronization drift between the physics simulation clock and the ROS2 offboard control nodes caused PID derivative spikes.
* **Solution:** Locked the simulation step to a deterministic 200 Hz physics rate while publishing synthetic sensor frames at 30 Hz with hardware timestamps (`sensor_msgs/Image`), stabilizing PID visual servoing without oscillations.

---

## 🛠️ Stack & Dependencies
* **Simulation:** NVIDIA Isaac Sim 4.0+ (Omniverse)
* **Autopilot Stack:** PX4 Autopilot (v1.14+), MicroXRCE-DDS Agent
* **Middleware:** ROS2 Humble Hawksbill
* **Communications:** MAVLink, pymavlink, pyzmq

---

## 🚀 Quickstart & Reproduction

### 1. Clone Repo & Install Requirements
```bash
git clone https://github.com/amanullah7x/uav-isaac-sim-digital-twin.git
cd uav-isaac-sim-digital-twin
pip install -r requirements.txt
```

### 2. Run Visual Servoing Simulation Controller
```bash
python ros2_offboard_tracker.py --duration 5.0
```

### 3. Launch with PX4 SITL and Isaac Sim
```bash
# Terminal 1: Launch PX4 SITL
make px4_sitl none_iris

# Terminal 2: Start micro-ROS agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Run offboard tracking node
python ros2_offboard_tracker.py
```
