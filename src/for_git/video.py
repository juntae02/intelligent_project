import cv2
import numpy as np

# 이미지 불러오기
image_path = "/home/juntae02/Pictures/fire.png"  # 원본 이미지 경로
image = cv2.imread(image_path)

# HSV 색공간으로 변환 (색상 분리를 위해)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 불꽃 색상 범위 지정 (노랑~주황 계열)
lower_fire = np.array([10, 100, 100])   # HSV 하한 (노란색)
upper_fire = np.array([35, 255, 255])   # HSV 상한 (주황색)

# 마스크 생성 (불꽃 영역만 흰색)
mask = cv2.inRange(hsv, lower_fire, upper_fire)

# 원본 이미지에서 불꽃 부분만 추출
fire = cv2.bitwise_and(image, image, mask=mask)

# 알파(투명도) 채널 생성: 불꽃 영역은 255(불투명), 나머지는 0(투명)
alpha = mask

# BGR 채널과 알파 채널 합치기 (BGRA)
b, g, r = cv2.split(fire)
rgba = cv2.merge((b, g, r, alpha))

# 결과 저장 (투명 배경 PNG)
output_path = "/home/juntae02/Pictures/fire_trans.png"
cv2.imwrite(output_path, rgba)

print(f"투명 배경 PNG 저장 완료: {output_path}")
