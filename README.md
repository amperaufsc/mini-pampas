# mini-PAMPAS

Autonomous driving stack for our RC crawler — a scaled-down testbed for the Driverless subteam's perception → planning → control pipeline, built by trainees as this season's onboarding challenge.

The goal: make the crawler drive itself around a mini track lined with 3D-printed cones, with no human input. Same architecture, same discipline as the full-size car, just smaller and faster to iterate on.

Tracked on the **[Driverless Challenge](https://github.com/orgs/amperaufsc/projects/8)** GitHub Project board — issues and PRs in this repo are linked there for cross-team visibility.

---

## Contents

- [Hardware](#hardware)
- [Track](#track)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Topic Interfaces](#topic-interfaces)
- [Modules](#modules)
- [Getting Started](#getting-started)
- [Tooling](#tooling)
- [Evaluation](#evaluation)
- [Deliverables](#deliverables)
- [Timeline](#timeline)

---

## Hardware

- **Drivetrain:** 4 wheels, front and back pairs originally each powered by an RC540 brushed motor with angular steering servos (servo rotates an arm connected to a bar that pushes/pulls the wheel pair sideways — bellcrank-style linkage). **Only the front pair is used for steering and traction** in this project; the rear pair is passive. Standard Ackermann-style kinematics apply.
- **Steering servo:** [Absima S90MH — 9kg/25T JR metal gear servo](https://www.modelsport.co.uk/product/absima-s90mh-9kg-25t-jr-metal-gear-servo-381461). Standard 3-pin JR connector (signal / +V / GND — one cavity in the 4-slot housing is unused by design). Operating voltage 4.8–6.6V — **needs its own regulated supply**, not the raw battery. Positional servo, ~1000–2000µs PWM range centered at ~1500µs.
- **Drive motor:** RC540 brushed motor on the front pair. Confirm whether an ESC is already present between battery/Pi and motor before assuming direct PWM control — a bare 540 only has 2 power leads.
- **Perception sensor:** [RPLIDAR A1M8](https://www.slamtec.com/en/Lidar/A1) — 360° 2D line-scan LiDAR, 0.15–12m range, up to 8000 samples/sec, USB/UART. Driver: [rplidar_ros](https://github.com/Slamtec/rplidar_ros.git) (`ros2` branch).
- **Compute:** [Raspberry Pi 4 Model B](https://www.raspberrypi.com/documentation/computers/processors.html#bcm2711).
- **Battery:** Zippy Compact 3000mAh 4S1P 20C LiPo (~14.8V nominal, ~16.8V full charge).
- **Mounting platform:** flat deck on top of the chassis for sensor + compute + battery.

**Power:** the 4S pack directly suits only the drive motor via ESC. Both the Pi and the servo need their own regulated 5V/5–6V supply (UBEC/BEC) — never wire either to the raw battery.

**Mounting:** the LiDAR, Raspberry Pi, and battery must be **securely fixed** to the platform (standoffs/screws, straps, printed mounts, zip-tie anchors) — not just resting on it. Vibration and sharp turns will shift anything not properly mounted. Plan and test this early.

### Track

The track is marked with **3D-printed miniature cones**, scaled to the crawler. Printing needs to happen early and in parallel with software work — perception tuning (size, shape, LiDAR reflectivity) depends on having real cones to test against, not mockups.

### Architecture

ROS2 nodes communicating over topics, visualized live via [Foxglove Studio](https://foxglove.dev/) (`foxglove_bridge`) — the same pattern the full-size car's stack uses.

```
LiDAR (/scan) → Perception (/perception/cones) → Planning (/planning/target_point) → Control (/cmd/drive) → Motor + Servo
                                                                                      ↑
                                                                        Sensors (/sensors/wheel_speed)
```

### Repository Structure
 
```
.
├── LICENSE
├── pyproject.toml
├── README.md
└── src
    └── mini_pampas
        ├── bringup       # launch files, starts all nodes together
        ├── control       # steering + throttle
        ├── __init__.py
        ├── perception    # LiDAR → cone clusters
        ├── planning      # cone clusters → target point
        └── sensors       # hall effect wheel speed, hardware interfacing
```

### Topic Interfaces

| Topic | Type | Published by | Consumed by |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `rplidar_ros` driver | `perception` |
| `/perception/cones` | cone positions (custom msg or `geometry_msgs/PoseArray`) | `perception` | `planning` |
| `/planning/target_point` | `geometry_msgs/PointStamped` | `planning` | `control` |
| `/cmd/drive` | steering angle + throttle (custom msg or `ackermann_msgs/AckermannDrive`) | `control` | motor/servo hardware interface |
| `/sensors/wheel_speed` | `std_msgs/Float32` or custom (one per driven wheel) | `sensors` | `control` (optional closed-loop), telemetry |

### Modules

#### `perception`
LiDAR scan → cone cluster positions. A **spatial** problem only: filter scan range/FOV, cluster points (distance/angle-gap thresholding is sufficient), compute centroid + distance per cluster. Frame-to-frame smoothing is explicitly out of scope here — that's `planning`'s job.
Verify early that the LiDAR's scan plane height actually intersects the cones given mounting height and cone size.
Refs: [RPLIDAR A1M8 datasheet](https://download-en.slamtec.com/api/download/rplidar-a1m8-datasheet/3.2?lang=en) · [rplidar_ros](https://docs.ros.org/en/humble/p/rplidar_ros/index.html) · [`sensor_msgs/LaserScan`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html)

#### `planning`
Cone positions → a single target point. The **temporal** half of the filtering split: smooth detections frame-to-frame (moving average/exponential smoothing), find the midpoint between left/right cone pairs, handle missing-cone edge cases (hold last valid point briefly).
Refs: [`geometry_msgs/PointStamped`](https://docs.ros.org/en/ros2_packages/humble/api/geometry_msgs/msg/PointStamped.html) · midpoint/pure-pursuit path following

#### `control`
Target point → steering + throttle. Front-only steering/traction means standard Ackermann/bicycle-model kinematics. Constant throttle is a fine baseline; closed-loop speed via hall sensors is a stretch goal.
Refs: [Absima S90MH spec](https://www.modelsport.co.uk/product/absima-s90mh-9kg-25t-jr-metal-gear-servo-381461) · `RPi.GPIO`/`pigpio` for PWM output

#### `hardware integration` (+ sensors)
Power, wiring, and mounting. Confirm motor/servo/ESC wiring, spec a BEC/UBEC for Pi + servo, get the LiDAR enumerating on the Pi, securely mount LiDAR/Pi/battery, wire hall effect sensors (one per driven wheel) with interrupt-based speed estimation into `/sensors/wheel_speed`. Document the wiring — this is the reference for next season.
Refs: [Raspberry Pi 4 Datasheet](https://pip-assets.raspberrypi.com/categories/545-raspberry-pi-4-model-b/documents/RP-008248-DS-1-bcm2711-peripherals.pdf) · UBEC/BEC basics

## Getting Started

```bash
git clone <repo-url>
cd mini_pampas
uv sync
colcon build
source install/setup.bash
ros2 launch mini_pampas bringup
```

> `uv` + `colcon` is not the standard ROS2 workflow — if dependency resolution fights with `colcon`/`ament_python`, fall back to a plain `venv` + `pip` setup and update this section accordingly.

## Tooling

- **ROS2** (`rclpy`) for all nodes and topic interfaces.
- **uv** for Python dependency management, `colcon build` for the ROS2 build.
- **Foxglove Studio** for live visualization of point clouds, cone detections, target point, and drive commands during development and demos.

## Evaluation

Each module is evaluated separately:

- **Perception:** clustering accuracy/robustness (false positives/negatives), performance on track edge cases
- **Planning:** path validity on curves, behavior with missing/noisy cone data
- **Control:** smoothness, respecting steering/speed limits, response to path changes
- **Sensors/Integration:** wiring safety/robustness, power stability under load, documentation quality

### Deliverables

1. Working code in your module, merged via the team's standard git/PR workflow.
2. A presentation using the team's template: what you built, key decisions/trade-offs, challenges, what you'd improve.
3. A recorded video presenting the project, archived for the team and next season's cohort.
4. If the car completes a successful run: a live demo to the rest of the team.

## Timeline

| Week | Focus |
|---|---|
| 3 | Kickoff, architecture walkthrough, ROS2 intro, teleop milestone |
| 4 | Module-specific foundations (wiring, kinematic model, LiDAR basics, path representation) |
| 5 | ROS2 deep dive (TF2, bag files), perception clustering |
| 6 | Perception → planning integration |
| 7 | Planning → control integration, first closed-loop driving |
| 8 | Full pipeline integration, first full lap attempt |
| 9 | Polish, bug fixing, presentations prepared |
| 10 | Buffer / deadline / live demo day |
