## Perception

This package is part of the autonomous driving stack for mini-pampas (an RC crawler platform). It implements the 2D LiDAR perception pipeline, taking raw laser scan data from an RPLiDAR A1M8 and outputting detected cone positions in the sensor's local coordinate frame.

## Hardware

Perception sensor: RPLIDAR A1M8 — 360° 2D line-scan LiDAR, 0.15–12m range, up to 8000 samples/sec, USB/UART. Driver: rplidar_ros (ros2 branch).

## Software

- Middleware: ROS 2 Humble
- Environment & Dependency Management: uv
- Containerization: Docker
- Visualization: Foxglove Studio (via foxglove_bridge)

## Processing Pipeline

The perception module treats cone detection as a purely spatial 2D clustering problem;

1. Filtering: Crop the raw 360° scan by minimum/maximum distance and angular Field of View (FOV) to discard noise, self-hits from the chassis, and irrelevant background geometry.

2. Polar-to-Cartesian Conversion: Project valid (range, angle) measurements into 2D (x, y) coordinates in the laser frame.

3. Clustering: Group consecutive points into candidate objects using Euclidean distance / angular-gap thresholding, filtering out clusters whose point count or physical width falls outside expected cone dimensions.

4. Centroid & Distance Estimation: Compute the geometric centroid (x, y) and Euclidean distance (sqrt(x^2 + y^2)) for each valid cone cluster.

## Structure

perception
├── perception
│   ├── cone_detector.py
│   └── __init__.py
└── ros
    └── perception_node.py

## perception_node and cone_detector

Following standard ROS 2 and Object-Oriented Programming (OOP) practices, core detection algorithms are decoupled from ROS 2 middleware communication.

- cone_detector.py: Framework-agnostic class (ConeDetector) that accepts raw range/angle arrays and thresholds, returning structured cone detections. Decoupled from ROS for straightforward unit testing.

- perception_node.py: Thin ROS 2 wrapper (PerceptionNode) that subscribes to /scan, passes scan data to ConeDetector, and publishes detected cone positions and visualization markers.

## ROS 2 interfaces

- Subscribed topics:
    - /scan: 
        - message type: sensor_msgs/msg/LaserScan
        - publisher: rplidar_ros
        - Description: Raw 360° 2D distance measurements and scan metadata (angle_min, angle_increment, ranges).

- Published topics:
    - /perception/cones:
        - message type: geometry_msgs/msg/PoseArray
        - description: Estimated 2D positions (x, y) of detected cones in the laser frame.

## Running

- Build the workspace:
    colcon build --packages-select perception
    source install/setup.bash

- Run the node: 
    ros2 launch /mini_pampas/src/mini_pampas/bringup/bringup_launch.py