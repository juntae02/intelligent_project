# 파일명: receive_and_rotate.py (robot3 네임스페이스에서 실행)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

class SignalReceiver(Node):
    def __init__(self):
        super().__init__('signal_receiver')
        self.subscription = self.create_subscription(
            Bool,
            'receive_signal',
            self.listener_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'📥 신호 수신: {msg.data}')
        if msg.data:
            self.rotate_robot()

    def rotate_robot(self, duration=3.0, speed=0.5):
        twist = Twist()
        twist.angular.z = speed
        self.get_logger().info('🔄 회전 시작!')
        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        while rclpy.ok():
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - start_time > duration:
                break
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('🛑 회전 종료')

def main(args=None):
    rclpy.init(args=args)
    node = SignalReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
