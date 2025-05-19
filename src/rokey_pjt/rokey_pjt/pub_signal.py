# 파일명: send_signal.py (robot2 네임스페이스에서 실행)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SignalPublisher(Node):
    def __init__(self):
        super().__init__('signal_publisher')
        self.publisher_ = self.create_publisher(Bool, '/robot3/receive_signal', 10)
        timer_period = 3.0  # 3초마다 신호 전송 (테스트용)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.signal_sent = False

    def timer_callback(self):
        if not self.signal_sent:
            msg = Bool()
            msg.data = True
            self.publisher_.publish(msg)
            self.get_logger().info('📡 신호 전송됨: True')
            self.signal_sent = True  # 한 번만 전송

def main(args=None):
    rclpy.init(args=args)
    node = SignalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
