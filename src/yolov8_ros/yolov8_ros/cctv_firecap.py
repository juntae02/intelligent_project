import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
from datetime import datetime
# ========================
# 설정값
# ========================
IMAGE_TOPIC = '/robot2/oakd/rgb/preview/image_raw'     # 이미지 토픽
SAVE_DIR = "img_capture"     # 이미지 저장 디렉토리
CAPTURE_KEY = 'c'            # 저장 키
EXIT_KEY = 'q'               # 종료 키
# ===============================
# ROS2 노드 클래스
# ===============================
class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.bridge = CvBridge()
        self.image = None
        # 디렉토리 준비
        os.makedirs(SAVE_DIR, exist_ok=True)
        self.prefix = input("저장할 파일의 접두어를 입력하세요: ").strip()
        self.image_count = 0
        # 구독자 설정
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )
        self.get_logger().info("이미지 수신 대기 중...")
        self.capture_loop()
    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
    def capture_loop(self):
        print("\n[안내] 'c' 키를 누르면 이미지가 저장됩니다.")
        print(f"[안내] '{EXIT_KEY}' 키를 누르면 프로그램이 종료됩니다.\n")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image is not None:
                cv2.imshow("ROBOT IMAGE", self.image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(CAPTURE_KEY):
                    filename = os.path.join(SAVE_DIR, f"{self.prefix}_img_{self.image_count}.jpg")
                    cv2.imwrite(filename, self.image)
                    print(f"이미지 저장 완료: {filename}")
                    self.image_count += 1
                elif key == ord(EXIT_KEY):
                    print("종료합니다.")
                    break
        cv2.destroyAllWindows()
# ===============================
# 실행 진입점
# ===============================
def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()