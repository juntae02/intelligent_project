import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
import math
import time
from std_msgs.msg import Bool  # ✅ 기존 퍼블리셔 유지
from example_interfaces.srv import Trigger  # ✅ 서비스 메시지 추가

class RotationController(Node):
    def __init__(self):
        super().__init__('rotation_controller')
        self._action_client = ActionClient(self, RotateAngle, '/robot2/rotate_angle')
        self._finish_publisher = self.create_publisher(Bool, '/robot2/action_finish', 10)

        # ✅ 서비스 서버 생성
        self._srv = self.create_service(Trigger, '/robot2/start_rotation', self.rotation_service_callback)

    def send_rotation_goal(self, angle_rad, speed_rad_per_sec):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle_rad
        goal_msg.max_rotation_speed = speed_rad_per_sec

        self._action_client.wait_for_server()   
        self.get_logger().info('Sending rotation goal...')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal was rejected by the server.')
            return

        self.get_logger().info('✅ Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'🎉 Rotation of {math.degrees(angle_rad):.1f}° complete.')

    def run_rotation_loop(self):
        angle_rad = math.radians(30)
        speed = 1.0  # rad/s
        count = 0

        self.get_logger().info('🔄 Rotating left 60°...')
        self.send_rotation_goal(angle_rad, speed)
        time.sleep(0.4)

        while (count < 5):
            self.get_logger().info('🔄 Rotating left 60°...')
            self.send_rotation_goal(-angle_rad*2, speed)
            time.sleep(0.4)

            self.get_logger().info('🔄 Rotating right 60°...')
            self.send_rotation_goal(angle_rad*2, speed)
            time.sleep(0.4)

            if count == 3:
                msg = Bool()
                msg.data = True
                self._finish_publisher.publish(msg)  # ✅ 기존 퍼블리셔 유지
            count += 1

        self.get_logger().info('🔄 Rotating left 60°...')
        self.send_rotation_goal(-angle_rad, speed)
        time.sleep(0.4)

        angle_rad = math.radians(360)
        self.get_logger().info('🔄 Rotating full 360° -> left ...')
        self.send_rotation_goal(angle_rad, speed)
        time.sleep(0.4)

        self.get_logger().info('🔄 Rotating full 360°-> right...')
        self.send_rotation_goal(-angle_rad, speed)
        time.sleep(0.4)

    # ✅ 서비스 콜백 함수 정의
    def rotation_service_callback(self, request, response):
        self.get_logger().info('📞 Rotation service triggered!')
        self.run_rotation_loop()
        response.success = True
        response.message = 'Rotation completed successfully.'
        return response


def main(args=None): 
    rclpy.init(args=args)
    controller = RotationController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
