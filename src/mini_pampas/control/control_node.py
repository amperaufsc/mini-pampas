import rclpy
from rclpy.node import Node
import math

from mini_pampas_msgs.msg import DriveCommand
from geometry_msg.msg import PointStamped
from mini_pampas.control.control import Control

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self._declare_parameter()

        self._control = self._make_control()

        self._publisher = self.create_publisher(DriveCommand, '/cmd/drive', 10)
        self._subscription = self.create_subscription(
            PointStamped,
            '/planning/target_point',
            self.control_callback,
            10
        )

    def _declare_parameter(self):
        self.declare_parameter('front_servo_pin', 17)
        self.declare_parameter('front_motor_forward_pin', 19)
        self.declare_parameter('front_motor_backward_pin', 20)
        self.declare_parameter('back_motor_forward_pin', 21)
        self.declare_parameter('back_motor_backward_pin', 22)
        self.declare_parameter('throttle_speed', 0.0)

    def _make_control(self):
        return Control(
            front_servo_pin=self.get_parameter('front_servo_pin').value,
            front_motor_forward_pin=self.get_parameter('front_motor_forward_pin').value,
            front_motor_backward_pin=self.get_parameter('front_motor_backward_pin').value,
            back_motor_forward_pin=self.get_parameter('back_motor_forward_pin').value,
            back_motor_backward_pin=self.get_parameter('back_motor_backward_pin').value
        )

    def _publish_command(self, throttle_speed, car_direction):
        msg = DriveCommand()

        car_direction = max(-25.0, min(car_direction, 25.0))
        msg.steering_angle = (math.radians(car_direction) * -1)

        throttle_speed = max(0.0, min(throttle_speed, 1.0))
        msg.throttle = throttle_speed

        self._publisher.publish(msg)

    def _get_direction(self, x, y):
        angle = math.atan2(y, x)
        return math.degrees(angle)

    def _get_distance(self, x, y):
        return math.dist((0, 0), (x, y))

    def control_callback(self, msg):
        x, y = msg.point.x, msg.point.y
        speed = float(self.get_parameter('throttle_speed').value)
        if self._get_distance(x, y) > 0.1:
            self._control.run_forward(speed)
        else:
            self._control.stop_car()
        direction = self._get_direction(x, y)
        self._control.set_car_direction(direction)

        self._publish_command(speed, direction)

    def close(self):
        self._control.close()


def main(args=None):
    rclpy.init(args=args)
    controlNode = ControlNode()
    try:
        rclpy.spin(controlNode)
    except KeyboardInterrupt:
        pass
    finally:
        controlNode.close()
        controlNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
