import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray

# Import your detector class from File 1
from .cone_detector import ConeDetector, DetectorConfig


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('perception')
        self._init_parameters()

        self.detector = ConeDetector()

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cones_pub = self.create_publisher(PoseArray, '/perception/cones', 10)
        self.get_logger().info('Perception node initialized and ready.')

    def _init_parameters(self) -> None:
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 4.00)
        self.declare_parameter('min_angle', math.pi / 2)
        self.declare_parameter('max_angle', -math.pi / 2)
        self.declare_parameter('cluster_thresh', 0.20)
        self.declare_parameter('min_cluster_pts', 2)
        self.declare_parameter('max_cluster_pts', 35)
        self.declare_parameter('max_cone_width', 0.30)

    def _get_config(self) -> DetectorConfig:
        return DetectorConfig(
            min_r=self.get_parameter('min_range').value,
            max_r=self.get_parameter('max_range').value,
            min_a=self.get_parameter('min_angle').value,
            max_a=self.get_parameter('max_angle').value,
            cluster_thresh=self.get_parameter('cluster_thresh').value,
            min_pts=self.get_parameter('min_cluster_pts').value,
            max_pts=self.get_parameter('max_cluster_pts').value,
            max_w=self.get_parameter('max_cone_width').value,
        )

    def scan_callback(self, msg: LaserScan) -> None:
        cfg = self._get_config()
        centroids = self.detector.process_scan(
            list(msg.ranges), msg.angle_min, msg.angle_increment, cfg
        )

        out_msg = PoseArray()
        out_msg.header = msg.header
        for cx, cy in centroids:
            pose = Pose()
            pose.position.x = float(cx)
            pose.position.y = float(cy)
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