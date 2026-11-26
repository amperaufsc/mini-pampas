import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import termios
import tty
import sys
import threading

move_bindings = {
    'w': (1.0, 0.0),
    'a': (0.0, -1.0),
    'd': (0.0, 1.0)
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def save_terminal_settings():
    return termios.tcsetattr(sys.stdin)

def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = save_terminal_settings()

    rclpy.init()
    
    node = rclpy.create_node('teleop_control')
    pub = node.create_publisher(PointStamped, '/planning/target_point', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.0
    y = 0.0
    z = 0.0

    msg = PointStamped()
    
    try:
        while  True:
            key = get_key(settings)
            if key in move_bindings.keys():
                x = move_bindings[key][0]
                y = move_bindings[key][1]
                z = 0.0
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                if (key == '\x03'):
                    break

            msg.point.x = x
            msg.point.y = y
            msg.point.z = z

            pub.publish(msg)
    except Exception as e:
        print(e)
    finally:
        x = 0.0
        y = 0.0
        z = 0.0
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        pub.publish(msg)
        rclpy.shutdown()
        spinner.join()
        restore_terminal_settings(settings)

if __name__ == '__main__':
    main()
