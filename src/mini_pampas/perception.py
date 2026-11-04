import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')

        # Subtask 2: Expose configurable ROS parameters for filtering ranges,
        # angle limits, and clustering thresholds.
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 4.00)
        self.declare_parameter('min_angle', math.pi / 2) # +90 deg
        self.declare_parameter('max_angle', -math.pi / 2) # -90 deg
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
    # 1. Fetch parameters
        min_r = self.get_parameter("min_range").value
        max_r = self.get_parameter("max_range").value
        min_a = self.get_parameter("min_angle").value
        max_a = self.get_parameter("max_angle").value
        cluster_thresh = self.get_parameter("cluster_thresh").value
        min_pts = self.get_parameter("min_cluster_pts").value
        max_pts = self.get_parameter("max_cluster_pts").value
        max_w = self.get_parameter("max_cone_width").value

        # 2. Filter & convert valid polar measurements to Cartesian coordinates (x, y)
        points = []
        angle = msg.angle_min

        for r in msg.ranges:
            # Handle both normal ranges [-pi/2, pi/2] and wrapped ranges [pi/2, -pi/2]
            in_angle = (min_a <= angle <= max_a) if (min_a <= max_a) else (angle >= min_a or angle <= max_a)

            if in_angle:
                if min_r <= r <= max_r and not math.isinf(r) and not math.isnan(r):
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append((x, y))
            angle += msg.angle_increment

        # 3. Distance-gap clustering
        clusters = []
        current_cluster = []

        for pt in points:
            if not current_cluster:
                current_cluster.append(pt)
                continue

            prev_pt = current_cluster[-1]
            dist = math.hypot(pt[0] - prev_pt[0], pt[1] - prev_pt[1])

            if dist <= cluster_thresh:
                current_cluster.append(pt)
            else:
                clusters.append(current_cluster)
                current_cluster = [pt]

        if current_cluster:
            clusters.append(current_cluster)

        # Merge first and last cluster if a cone straddles the -pi / +pi boundary
        if len(clusters) > 1:
            first_pt = clusters[0][0]
            last_pt = clusters[-1][-1]
            if math.hypot(first_pt[0] - last_pt[0], first_pt[1] - last_pt[1]) <= cluster_thresh:
                clusters[0] = clusters[-1] + clusters[0]
                clusters.pop()
                
        # 4. Filter clusters by size and compute centroids
        out_msg = PoseArray()
        out_msg.header = msg.header

        for cluster in clusters:
            num_pts = len(cluster)
            if not (min_pts <= num_pts <= max_pts):
                continue

            cluster_width = math.hypot(
                cluster[-1][0] - cluster[0][0], cluster[-1][1] - cluster[0][1]
            )
            if cluster_width > max_w:
                continue

            # Centroid calculation
            cx = sum(p[0] for p in cluster) / num_pts
            cy = sum(p[1] for p in cluster) / num_pts

            pose = Pose()
            pose.position.x = float(cx)
            pose.position.y = float(cy)
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            out_msg.poses.append(pose)

        # 5. Publish to /perception/cones
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