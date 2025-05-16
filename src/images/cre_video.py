import cv2
import numpy as np

# 입력 불꽃 영상 (GIF도 OpenCV로 읽히긴 하지만 프레임 수 제한이 있을 수 있음)
fire_video_path = "/home/juntae02/Videos/fire.gif"

# 배경 이미지 불러오기
background = cv2.imread("/home/juntae02/Pictures/datacenter.png")
bg_h, bg_w = background.shape[:2]

# 동영상 열기
cap = cv2.VideoCapture(fire_video_path)
fps = cap.get(cv2.CAP_PROP_FPS)
if fps == 0:
    fps = 25  # 기본 fps 설정 (GIF는 fps가 0일 수 있음)

# 출력 영상 설정 (배경 이미지 크기와 동일)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter("/home/juntae02/Videos/output_fire.mp4", fourcc, fps, (bg_w, bg_h))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 불꽃 추출: HSV 변환 및 마스크 생성
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_fire = np.array([10, 100, 100])
    upper_fire = np.array([35, 255, 255])
    mask = cv2.inRange(hsv, lower_fire, upper_fire)

    # 알파 채널 생성 (float)
    alpha = mask.astype(float) / 255

    # 불꽃 프레임에서 불꽃 부분만 (마스크 적용)
    fire_bgr = cv2.bitwise_and(frame, frame, mask=mask)

    # 배경 이미지 복사 (float 타입)
    composed = background.copy().astype(float)

    # 불꽃 합성 위치 (좌측 상단)
    x_offset, y_offset = 0, 0
    h, w = fire_bgr.shape[:2]

    # 배경 크기 넘지 않도록 크기 조정
    if y_offset + h > bg_h:
        h = bg_h - y_offset
    if x_offset + w > bg_w:
        w = bg_w - x_offset

    # roi 재설정 (배경에서 합성할 영역)
    roi = composed[y_offset:y_offset+h, x_offset:x_offset+w]

    # fire_bgr와 alpha도 크기 맞추기 (crop)
    fire_bgr_cropped = fire_bgr[0:h, 0:w]
    alpha_cropped = alpha[0:h, 0:w]

    # 알파 채널을 3채널로 확장 (for broadcasting)
    alpha_c3 = np.dstack([alpha_cropped]*3)

    # 알파 블렌딩 (float 계산)
    roi[:] = alpha_c3 * fire_bgr_cropped + (1 - alpha_c3) * roi

    # 합성 영역 배경에 덮어쓰기
    composed[y_offset:y_offset+h, x_offset:x_offset+w] = roi

    # uint8로 변환 후 저장
    composed = composed.astype(np.uint8)
    out.write(composed)

cap.release()
out.release()
print("배경 합성 완료된 동영상 저장됨: output_fire_on_bg.mp4")
