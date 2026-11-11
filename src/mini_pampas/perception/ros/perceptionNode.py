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

        # 1. Declare & cache parameters (includes minor optmization to avoid repeated get_parameter calls that could overload the raspberry pi)
        self._init_parameters()
        self.cfg = self._load_config()
        self.add_on_set_parameters_callback(self._on_params_changed)

        # 2. Initialize stateful detector
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

        self.get_logger().info('Perception node initialized with shape check & multi-frame tracker.')

    def _init_parameters(self) -> None:
        # Base parameters
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 2.00)
        self.declare_parameter('min_angle', math.pi / 2)
        self.declare_parameter('max_angle', -math.pi / 2)
        self.declare_parameter('cluster_thresh', 0.12)
        self.declare_parameter('min_cluster_pts', 2)
        self.declare_parameter('max_cluster_pts', 35)
        self.declare_parameter('max_cone_width', 0.20) # Needs to be measured and adjusted (smaller value is expected)

        # Upgrade 1 & 2 parameters
        self.declare_parameter('cone_radius', 0.045) # Needs to be measured and adjusted (smaller value is expected)
        self.declare_parameter('expected_beam_ratio', 0.35)

        # Upgrade 3 parameters (Geometric Shape Check)
        self.declare_parameter('max_cone_depth', 0.18)
        self.declare_parameter('convexity_tol', 0.025)

        # Upgrade 4 parameters (Temporal Tracking)
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('assoc_dist', 0.30)
        self.declare_parameter('min_hits', 2)
        self.declare_parameter('max_misses', 1)
        self.declare_parameter('ema_alpha', 0.85)

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
            enable_tracking=self.get_parameter('enable_tracking').value,
            assoc_dist=self.get_parameter('assoc_dist').value,
            min_hits=self.get_parameter('min_hits').value,
            max_misses=self.get_parameter('max_misses').value,
            ema_alpha=self.get_parameter('ema_alpha').value,
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
                self.cfg.expected_beam_ratio = p.value
            elif p.name == 'max_cone_depth':
                self.cfg.max_depth = p.value
            elif p.name == 'convexity_tol':
                self.cfg.convexity_tol = p.value
            elif p.name == 'enable_tracking':
                self.cfg.enable_tracking = p.value
            elif p.name == 'assoc_dist':
                self.cfg.assoc_dist = p.value
            elif p.name == 'min_hits':
                self.cfg.min_hits = p.value
            elif p.name == 'max_misses':
                self.cfg.max_misses = p.value
            elif p.name == 'ema_alpha':
                self.cfg.ema_alpha = p.value
        return SetParametersResult(successful=True)

    def scan_callback(self, msg: LaserScan) -> None:
        # Pass msg.ranges directly without copying to a list
        centroids = self.detector.process_scan(
            msg.ranges, msg.angle_min, msg.angle_increment, self.cfg
        )

        out_msg = PoseArray()
        out_msg.header = msg.header

        for cx, cy in centroids:
            pose = Pose()
            pose.position.x = float(cx)
            pose.position.y = float(cy)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            out_msg.poses.append(pose)

        self.cones_pub.publish(out_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()