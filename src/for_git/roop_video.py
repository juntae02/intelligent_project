import cv2
import numpy as np
import asyncio
import platform
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# 파일 경로
VIDEO1_PATH = "output_fire.mp4"
IMAGE2_PATH = "datacenter.png"
OUTPUT_PATH = "output_frame.png"

# 전역 변수로 모드 관리
current_mode = 2
auto_switched = False
action_finish_received = False

# ROS2 노드 클래스 정의 (토픽 이름 변경됨)
class ActionFinishSubscriber(Node):
    def __init__(self):
        super().__init__('action_finish_listener')
        self.subscription = self.create_subscription(
            Bool,
            'action_finish',  
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global current_mode, action_finish_received
        if msg.data:
            self.get_logger().info('Received finish_fire=True. Switching to datacenter image.')
            current_mode = 2
            action_finish_received = True

# ROS2 노드 실행 함수 (백그라운드)
def run_ros2_node():
    rclpy.init()
    node = ActionFinishSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# 메인 OpenCV 루프
async def main():
    global current_mode, auto_switched, action_finish_received

    cap1 = cv2.VideoCapture(VIDEO1_PATH)
    img2 = cv2.imread(IMAGE2_PATH)

    if not cap1.isOpened():
        print("Error: Could not open video file.")
        return
    if img2 is None:
        print("Error: Could not open image file.")
        return

    start_time = time.time()

    if platform.system() != "Emscripten":
        screen_width, screen_height = 1920, 1080
        try:
            import pyautogui
            screen_width, screen_height = pyautogui.size()
        except ImportError:
            print("PyAutoGUI not installed, using default 1920x1080")

        cv2.namedWindow("Video Player", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Video Player", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    FPS = 30

    while True:
        elapsed = time.time() - start_time

        # 10초 후 자동 전환
        if elapsed > 10 and not auto_switched:
            current_mode = 1
            cap1.set(cv2.CAP_PROP_POS_FRAMES, 0)
            auto_switched = True
            print("Auto-switched to fire video after 10 seconds")

        if current_mode == 1:
            ret, frame = cap1.read()
            if not ret:
                cap1.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = cap1.read()
        else:
            frame = img2.copy()

        if platform.system() != "Emscripten":
            frame = cv2.resize(frame, (screen_width, screen_height), interpolation=cv2.INTER_AREA)
            cv2.imshow("Video Player", frame)

        cv2.imwrite(OUTPUT_PATH, frame)

        key = -1
        if platform.system() != "Emscripten":
            key = cv2.waitKey(1) & 0xFF

        if key == ord('1'):
            if current_mode != 1:
                current_mode = 1
                cap1.set(cv2.CAP_PROP_POS_FRAMES, 0)
                print("Manually switched to fire video")
        elif key == ord('2'):
            if current_mode != 2:
                current_mode = 2
                print("Manually switched to datacenter image")
        elif key == ord('q'):
            break

        await asyncio.sleep(1.0 / FPS)

    cap1.release()
    if platform.system() != "Emscripten":
        cv2.destroyAllWindows()

# 메인 실행
if platform.system() != "Emscripten":
    if __name__ == "__main__":
        # ROS2 노드를 별도 스레드로 실행
        ros_thread = threading.Thread(target=run_ros2_node, daemon=True)
        ros_thread.start()

        # OpenCV 이벤트 루프 실행
        asyncio.run(main())
