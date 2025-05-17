import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from example_interfaces.srv import Trigger  # 서비스 메시지 타입
from geometry_msgs.msg import Twist  # 이동 명령 메시지

# 초기 위치 및 목표 위치 정의
INITIAL_POSE_POSITION = [0.0, 0.0]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

GOAL_POSES = [
    ([-1.95, -0.068], TurtleBot4Directions.WEST),
]

class Nav2WithRotationClient(Node):
    def __init__(self):
        super().__init__('nav2_with_rotation_client')
        self.cli = self.create_client(Trigger, 'start_rotation')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for rotation service...')
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def call_rotation_service(self):
        req = Trigger.Request()
        self.get_logger().info('📞 Calling rotation service...')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'✅ Rotation completed: {future.result().success}')
        else:
            self.get_logger().error('❌ Rotation service call failed')

    def move_forward_1m(self):
        self.get_logger().info('➡️ Moving forward 1 meter...')
        twist = Twist()
        twist.linear.x = 0.2  # 0.2 m/s 전진 속도 (필요에 따라 조절)
        duration_sec = 5  # 1m 이동 = 속도 0.2m/s * 5초

        start_time = self.get_clock().now()
        rate = self.create_rate(10)  # 10Hz 제어 주기

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration_sec:
            self._cmd_vel_pub.publish(twist)
            rate.sleep()

        # 정지 명령 보내기
        twist.linear.x = 0.0
        self._cmd_vel_pub.publish(twist)
        self.get_logger().info('🛑 Movement stopped.')

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()
    nav_and_rotate_node = Nav2WithRotationClient()

    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped(INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()
    navigator.undock()

    goal_pose_msgs = [navigator.getPoseStamped(position, direction) for position, direction in GOAL_POSES]
    navigator.startFollowWaypoints(goal_pose_msgs)
    # navigator.waitUntilNavigationComplete()  # 🔧

    # 목표 지점 도착 후 회전 서비스 호출
    nav_and_rotate_node.call_rotation_service()

    # 회전 완료 후 앞으로 1m 전진
    nav_and_rotate_node.move_forward_1m()

    rclpy.shutdown()

if __name__ == '__main__':
    main()