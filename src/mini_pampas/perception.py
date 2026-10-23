import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
# If your team wants ConeArray instead, replace the line above with:
# from mini_pampas_msgs.msg import ConeArray


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')

        # Subtask 2: Expose configurable ROS parameters for filtering ranges,
        # angle limits, and clustering thresholds.
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 4.00)
        self.declare_parameter('min_angle', -math.pi / 2)
        self.declare_parameter('max_angle', math.pi / 2)
        self.declare_parameter('cluster_thresh', 0.20)
        self.declare_parameter('min_cluster_pts', 2)
        self.declare_parameter('max_cluster_pts', 35)
        self.declare_parameter('max_cone_width', 0.30)

        # Subscribe to /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Subtask 1: Create a publisher for /perception/cones using geometry_msgs/msg/PoseArray
        self.cones_pub = self.create_publisher(
            PoseArray,
            '/perception/cones',
            10
        )

        self.get_logger().info('Perception node initialized and ready.')

    def scan_callback(self, msg: LaserScan):
        # Definition of Done: Publish an empty/dummy message onto /perception/cones
        # at matching frequencies when receiving /scan messages.
        out_msg = PoseArray()
        out_msg.header = msg.header  # Forward timestamp and frame_id
        self.cones_pub.publish(out_msg)


def main(args=None):
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