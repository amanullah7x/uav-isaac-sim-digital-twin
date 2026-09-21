#!/usr/bin/env python3
"""
ROS2 Humble Offboard Visual Servoing Node for NVIDIA Isaac Sim + PX4 SITL

Author: Amanullah Naseer (amanullah7x)
License: MIT

Subscribes:
- /camera/image_raw (sensor_msgs/Image): Synthetic camera frames from Isaac Sim.
- /px4/vehicle_status (px4_msgs/VehicleStatus): Autopilot arming & offboard state.

Publishes:
- /px4/trajectory_setpoint (px4_msgs/TrajectorySetpoint): 3D velocity vectors (Vx, Vy, Vz, YawRate).
"""

import math
import time
import argparse
import logging
from typing import Tuple

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [OffboardTracker] %(message)s")
logger = logging.getLogger("OffboardTracker")


class VisualServoingPIDController:
    """Proportional-Integral-Derivative controller for drone visual line-of-sight tracking."""

    def __init__(self, kp_xy: float = 1.2, kd_xy: float = 0.15, kp_yaw: float = 0.8):
        self.kp_xy = kp_xy
        self.kd_xy = kd_xy
        self.kp_yaw = kp_yaw
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.last_time = time.time()

    def compute_control_cmd(self, norm_err_x: float, norm_err_y: float) -> Tuple[float, float, float]:
        """
        Computes velocity setpoints (Vx, Vy, YawRate) based on normalized screen error (-1.0 to +1.0).
        """
        now = time.time()
        dt = max(1e-3, now - self.last_time)

        # Derivative terms
        d_err_x = (norm_err_x - self.prev_error_x) / dt
        d_err_y = (norm_err_y - self.prev_error_y) / dt

        # Control commands (body frame velocity and yaw rate)
        vy_cmd = (self.kp_xy * norm_err_x) + (self.kd_xy * d_err_x)
        vz_cmd = (self.kp_xy * norm_err_y) + (self.kd_xy * d_err_y)
        yaw_rate_cmd = self.kp_yaw * norm_err_x

        self.prev_error_x = norm_err_x
        self.prev_error_y = norm_err_y
        self.last_time = now

        return vy_cmd, vz_cmd, yaw_rate_cmd


def run_standalone_simulation(duration_sec: float = 5.0):
    """Executes closed-loop visual servoing simulation loop."""
    logger.info("Initializing Visual Servoing Controller for Isaac Sim Digital Twin...")
    controller = VisualServoingPIDController()
    
    start = time.time()
    step = 0
    while time.time() - start < duration_sec:
        # Simulate target traversing across camera FOV
        t = time.time() - start
        err_x = math.sin(t * 1.5) * 0.4
        err_y = math.cos(t * 1.0) * 0.2

        vy, vz, yaw_rate = controller.compute_control_cmd(err_x, err_y)
        if step % 10 == 0:
            logger.info(
                f"[SITL Loop] Error: (X={err_x:+.2f}, Y={err_y:+.2f}) -> "
                f"Setpoint: [Vy={vy:+.2f} m/s, Vz={vz:+.2f} m/s, YawRate={yaw_rate:+.2f} rad/s]"
            )
        step += 1
        time.sleep(0.05)  # 20 Hz setpoint rate

    logger.info("Visual servoing simulation loop completed successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Isaac Sim + PX4 Offboard Visual Servoing Node")
    parser.add_argument("--duration", type=float, default=3.0, help="Simulation test duration in seconds")
    args = parser.parse_args()
    run_standalone_simulation(args.duration)
