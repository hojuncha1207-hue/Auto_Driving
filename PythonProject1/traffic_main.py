import cv2
import serial
import time
import numpy as np
from traffic_library import Traffic_light_Detector

# --- [사용자 설정 변수] ---
# 아두이노 포트는 본인 환경에 맞춰주세요 (COM6)
ARDUINO_PORT = 'COM6'
STOP_RADIUS_THRESHOLD = 30

# --- [초기화] ---
try:
    ser = serial.Serial(ARDUINO_PORT, 9600, timeout=0.1)
    time.sleep(2)
    print("Arduino Connected")
except:
    print("Arduino Connection Failed")
    ser = None

detector = Traffic_light_Detector()
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)  # 2번 카메라


def main():
    print(f"시스템 시작 (정지 임계치: {STOP_RADIUS_THRESHOLD})")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break

        # 1. 신호등 데이터 읽기
        color, mask, radius = detector.object_detection(frame)

        # 2. 제어 로직
        steering_angle = 0.0
        speed_mode = 0
        status_msg = "DRIVING"

        if color == "RED":
            if radius >= STOP_RADIUS_THRESHOLD:
                speed_mode = 1  # 정지
                status_msg = f"STOP (Radius: {radius})"
            else:
                speed_mode = 0  # 주행 중 (아직 멀음)
                status_msg = f"RED LIGHT FAR (Radius: {radius})"
        else:
            status_msg = "PATH CLEAR / GREEN"

        # 3. 아두이노 전송
        if ser:
            data = f"{steering_angle},{speed_mode}\n"
            ser.write(data.encode())

        # 4. 모니터링 화면 출력
        display_frame = frame.copy()
        cv2.putText(display_frame, status_msg, (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Steer: {steering_angle} | Mode: {speed_mode}", (15, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # 반지름 시각화 (인식된 크기 확인용)
        if radius > 0:
            cv2.circle(display_frame, (frame.shape[1] // 2, frame.shape[0] // 2), radius, (255, 255, 0), 2)

        cv2.imshow("Main Driving System", display_frame)
        if mask is not None:
            cv2.imshow("Traffic Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 종료 처리q
    cap.release()
    cv2.destroyAllWindows()
    if ser: ser.close()
    print("프로그램 종료")


if __name__ == "__main__":
    main()