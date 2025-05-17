import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import threading
import os
import sys
import tf2_ros
import tf2_geometry_msgs


YOLO_MODEL_PATH = '/home/park/rokey_ws/model/yololv8n_best.pt'
RGB_TOPIC = '/robot2/oakd/rgb/preview/image_raw'
DEPTH_TOPIC = '/robot2/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = '/robot2/oakd/stereo/camera_info'
TARGET_CLASS_ID = 0  # ì˜ˆ: car


class YoloDepthDistance(Node):
    def __init__(self):
        super().__init__('yolo_depth_distance')
        self.get_logger().info("YOLO + Depth ê±°ë¦¬ ì¶œë ¥ ë…¸ë“œ ì‹œìž‘")

        if not os.path.exists(YOLO_MODEL_PATH):
            self.get_logger().error(f"YOLO ëª¨ë¸ì´ ì¡´ìž¬í•˜ì§€ ì•ŠìŠµë‹ˆë‹¤: {YOLO_MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(YOLO_MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])

        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.lock = threading.Lock()

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)
        self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo ìˆ˜ì‹  ì™„ë£Œ")

    def rgb_callback(self, msg):
        with self.lock:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def processing_loop(self):
        cv2.namedWindow("YOLO Distance View", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                if self.rgb_image is None or self.depth_image is None or self.K is None:
                    continue
                rgb = self.rgb_image.copy()
                depth = self.depth_image.copy()

            results = self.model(rgb, stream=True)
            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls != TARGET_CLASS_ID:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2

                    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
                        continue

                    val = depth[v, u]
                    distance_m = val / 1000.0 if depth.dtype == np.uint16 else float(val)

                    label = self.class_names[cls] if cls < len(self.class_names) else f'class_{cls}'
                    # self.get_logger().info(f"{label} at pixel ({u},{v}) â†’ {distance_m:.2f}m")

                    # ì¤‘ì‹¬ í”½ì…€ ê¸°ì¤€ìœ¼ë¡œ ì¹´ë©”ë¼ ì¢Œí‘œê³„ì˜ í¬ì¸íŠ¸ ê³„ì‚°
                    X_cam = distance_m 
                    # Y_cam = (v - self.K[1, 2]) * distance_m / self.K[1, 1]
                    # Z_cam = distance_m

                    point_camera = PointStamped()
                    point_camera.header.stamp = self.get_clock().now().to_msg()
                    point_camera.header.frame_id = 'oakd_rgb_camera_frame'
                    point_camera.point.x = X_cam
                    # point_camera.point.y = Y_cam
                    # point_camera.point.z = Z_cam
                    self.get_logger().info(
                            f"Camera Link: ({point_camera.point.x:.2f})")

                    try:
                        point_map = self.tf_buffer.transform(
                            point_camera,
                            'map',
                            timeout=rclpy.duration.Duration(seconds=0.5)
                        )
                        self.get_logger().info(
                            f"Map Coordinates: ({point_map.point.x:.2f})"
                        )
                        self.get_logger().info(
                            f"Camera Link: ({point_camera.point.x:.2f})"
                        )
                    except Exception as e:
                        self.get_logger().warn(f"TF ë³€í™˜ ì‹¤íŒ¨: {e}")

                    # ì‹œê°í™”
                    cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(rgb, (u, v), 4, (0, 0, 255), -1)
                    cv2.putText(rgb, f"{distance_m:.2f}m", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            cv2.imshow("YOLO Distance View", rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


def main():
    rclpy.init()
    node = YoloDepthDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()