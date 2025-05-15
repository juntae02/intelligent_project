from ultralytics import YOLO
import cv2

# 학습된 모델 로드
model = YOLO("/home/juntae02/Desktop/best.pt")

# 카메라 스트림 열기 (카메라 인덱스 0, 필요 시 변경)
cap = cv2.VideoCapture(2)


while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # 160x160으로 리사이징
    frame_resized = cv2.resize(frame, (160, 160))
    
    # 프레임 전처리: 밝기/대비 조정
    frame_resized = cv2.convertScaleAbs(frame_resized, alpha=1.2, beta=10)
    
    # 예측
    results = model.predict(frame_resized, conf=0.05, iou=0.5, device=1)
    
    # 결과 시각화
    annotated_frame = results[0].plot()
    cv2.imshow("Fire Detection", annotated_frame)
    
    # 디버깅: 탐지된 객체의 Confidence 출력
    for r in results:
        for box in r.boxes:
            print(f"Confidence: {box.conf.item():.3f}, Class: {box.cls.item()}")
    
    # 'q' 키로 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
##
cap.release()
cv2.destroyAllWindows()
 


 
# import cv2

# # 카메라 장치 열기 (기본 웹캠: 0번 장치)
# cap = cv2.VideoCapture(2)

# if not cap.isOpened():
#     print("❌ 카메라를 열 수 없습니다.")
#     exit()

# print("✅ 카메라가 정상적으로 연결되었습니다. 'q'를 눌러 종료하세요.")

# while True:
#     ret, frame = cap.read()  # 프레임 읽기
#     if not ret:
#         print("⚠️ 프레임을 읽을 수 없습니다.")
#         break

#     cv2.imshow('Camera Test', frame)  # 화면에 프레임 보여주기

#     if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q'를 누르면 종료
#         break

# cap.release()
# cv2.destroyAllWindows()