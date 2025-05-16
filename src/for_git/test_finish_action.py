import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class ActionFinishPublisher(Node):
    def __init__(self):
        super().__init__('action_finish_publisher')
        self.publisher_ = self.create_publisher(Bool, 'action_finish', 10)

    def publish_action_finish(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info('Sent action_finish: True')

def main(args=None):
    rclpy.init(args=args)
    node = ActionFinishPublisher()

    # 1초 대기 후 전송
    time.sleep(1.0)
    node.publish_action_finish()

    # 조금 기다렸다가 종료
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
