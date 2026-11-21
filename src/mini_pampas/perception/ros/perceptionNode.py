import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray

try:
    from .coneDetector import ConeDetector, DetectorConfig
except ImportError:
    from coneDetector import ConeDetector, DetectorConfig


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('perception')

        # 1. Declare & cache parameters (avoids repeated get_parameter calls on the Raspberry Pi)
        self._init_parameters()
        self.cfg = self._load_config()
        self.add_on_set_parameters_callback(self._on_params_changed)

        # 2. Initialize stateless detector
        self.detector = ConeDetector()

        # 3. Setup ROS 2 pub/sub
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cones_pub = self.create_publisher(
            PoseArray,
            '/perception/cones',
            10
        )

        self.get_logger().info('Perception node initialized (stateless spatial detector).')

    def _init_parameters(self) -> None:
        # Base parameters
        self.declare_parameter('min_range', 0.05)       # Don't clip ultra-close cones (<15 cm)
        self.declare_parameter('max_range', 1.50)
        self.declare_parameter('min_angle', -math.pi)   # Avoid clipping cones near -pi/2 boundary
        self.declare_parameter('max_angle', math.pi)
        self.declare_parameter('cluster_thresh', 0.08)
        self.declare_parameter('min_cluster_pts', 2)
        self.declare_parameter('max_cluster_pts', 60)   # High beam density at close range
        self.declare_parameter('max_cone_width', 0.20)

        # Upgrade 1 & 2 parameters
        self.declare_parameter('cone_radius', 0.045)
        self.declare_parameter('expected_beam_ratio', 0.10)  # Lowered for partially occluded cones

        # Upgrade 3 parameters (Geometric Shape Check)
        self.declare_parameter('max_cone_depth', 0.18)
        self.declare_parameter('convexity_tol', 0.04)

    def _load_config(self) -> DetectorConfig:
        return DetectorConfig(
            min_r=self.get_parameter('min_range').value,
            max_r=self.get_parameter('max_range').value,
            min_a=self.get_parameter('min_angle').value,
            max_a=self.get_parameter('max_angle').value,
            cluster_thresh=self.get_parameter('cluster_thresh').value,
            min_pts=self.get_parameter('min_cluster_pts').value,
            max_pts=self.get_parameter('max_cluster_pts').value,
            max_w=self.get_parameter('max_cone_width').value,
            cone_radius=self.get_parameter('cone_radius').value,
            expected_beam_ratio=self.get_parameter('expected_beam_ratio').value,
            max_depth=self.get_parameter('max_cone_depth').value,
            convexity_tol=self.get_parameter('convexity_tol').value,
        )

    def _on_params_changed(self, params: List[Parameter]) -> SetParametersResult:
        """Updates cached config only when a parameter is changed via CLI/rqt."""
        for p in params:
            if p.name == 'min_range':
                self.cfg.min_r = p.value
            elif p.name == 'max_range':
                self.cfg.max_r = p.value
            elif p.name == 'min_angle':
                self.cfg.min_a = p.value
            elif p.name == 'max_angle':
                self.cfg.max_a = p.value
            elif p.name == 'cluster_thresh':
                self.cfg.cluster_thresh = p.value
            elif p.name == 'min_cluster_pts':
                self.cfg.min_pts = p.value
            elif p.name == 'max_cluster_pts':
                self.cfg.max_pts = p.value
            elif p.name == 'max_cone_width':
                self.cfg.max_w = p.value
            elif p.name == 'cone_radius':
                self.cfg.cone_radius = p.value
            elif p.name == 'expected_beam_ratio':
                self.cfg.