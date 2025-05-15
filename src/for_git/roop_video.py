## 추가로 로봇의 액션이 끝나면 토픽을 받아서, 다시 2번 이미지로 변환

import cv2
import numpy as np
import asyncio
import platform
import time

# 파일 경로
VIDEO1_PATH = "output_fire.mp4"
IMAGE2_PATH = "datacenter.png"
OUTPUT_PATH = "output_frame.png"

async def main():
    cap1 = cv2.VideoCapture(VIDEO1_PATH)
    img2 = cv2.imread(IMAGE2_PATH)
    
    if not cap1.isOpened():
        print("Error: Could not open video file.")
        return
    if img2 is None:
        print("Error: Could not open image file.")
        return

    current_mode = 2  # 시작은 이미지 모드로 설정
    auto_switched = False
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

        # 10초가 지나고 아직 자동 전환 안 했을 경우
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

        # 수동 키 입력 처리
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

if platform.system() == "Emscripten":
    asyncio.ensure_future(main())
else:
    if __name__ == "__main__":
        asyncio.run(main())
