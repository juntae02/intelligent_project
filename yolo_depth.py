import cv2
import numpy as np

# YOLO 모델 로드 (예: YOLOv5)
model = cv2.dnn.readNet("yolov5.weights", "yolov5.cfg")
layer_names = model.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in model.getUnconnectedOutLayers()]

# RealSense 등에서 받은 정렬된 RGB + 뎁스 이미지
rgb_image = cv2.imread("rgb.png")
depth_image = cv2.imread("depth.png", cv2.IMREAD_UNCHANGED)  # 16-bit or float32 뎁스

depth_scale = 0.001  # 단위 변환: mm → m, 장비에 따라 다름

height, width = rgb_image.shape[:2]
blob = cv2.dnn.blobFromImage(rgb_image, 1/255.0, (416, 416), swapRB=True, crop=False)
model.setInput(blob)
outputs = model.forward(output_layers)

boxes = []
confidences = []
class_ids = []

for output in outputs:
    for detection in output:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5:
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)

            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

for i in indices.flatten():
    x, y, w, h = boxes[i]
    label = str(class_ids[i])
    cx, cy = x + w // 2, y + h // 2

    # 중심점 거리 측정
    if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
        depth_value = depth_image[cy, cx]
        distance = depth_value * depth_scale
    else:
        distance = -1

    # 바운딩 박스 내 평균 거리 측정 (더 정밀)
    roi = depth_image[max(0, y):min(y+h, depth_image.shape[0]), max(0, x):min(x+w, depth_image.shape[1])]
    valid = roi[roi > 0]
    if len(valid) > 0:
        avg_distance = np.mean(valid) * depth_scale
    else:
        avg_distance = -1

    print(f"[{label}] 중심점 거리: {distance:.2f} m, 평균 거리: {avg_distance:.2f} m")

    cv2.rectangle(rgb_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.putText(rgb_image, f"{avg_distance:.2f} m", (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

cv2.imshow("Detection with Distance", rgb_image)
cv2.waitKey(0)
cv2.destroyAllWindows()