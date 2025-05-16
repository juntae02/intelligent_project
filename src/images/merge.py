import cv2
import numpy as np

# 배경 이미지 불러오기 (RGB)
background = cv2.imread("/home/juntae02/Pictures/datacenter.png")

# 불꽃 이미지 불러오기 (RGBA)
fire = cv2.imread("/home/juntae02/Pictures/fire_trans.png", cv2.IMREAD_UNCHANGED)

# 불꽃 이미지 크기 조절 (필요하면)
fire = cv2.resize(fire, (300, 300))

# 불꽃 이미지 채널 분리
b_fire, g_fire, r_fire, alpha_fire = cv2.split(fire)

# 알파 채널 0~1 범위로 정규화
alpha_fire = alpha_fire.astype(float) / 255

# 배경 이미지에서 불꽃을 합성할 위치 지정 (좌측 상단 좌표)
x_offset = 40
y_offset = 90

# 배경 이미지에서 불꽃이 들어갈 영역 슬라이싱
h, w = fire.shape[:2]
roi = background[y_offset:y_offset+h, x_offset:x_offset+w]

# 배경과 불꽃 각각에 알파값 곱해서 합성
for c in range(3):
    roi[:, :, c] = (alpha_fire * fire[:, :, c] + (1 - alpha_fire) * roi[:, :, c])

# 합성된 영역을 배경 이미지에 덮어쓰기
background[y_offset:y_offset+h, x_offset:x_offset+w] = roi

# 결과 저장
cv2.imwrite("/home/juntae02/Pictures/merge_image.png", background)

print("배경 이미지에 불꽃 합성 완료! 저장됨: composited_image.png")
