import serial
import time
import cv2
import numpy as np
from obstacle_library import LidarObjectDetector  # 주신 코드를 이 파일명으로 저장하세요

# --- 설정부 ---
ARDUINO_PORT = 'COM6'  # 아두이노 포트 (본인 환경에 맞게 수정)
LIDAR_PORT = 'COM5'  # LiDAR 포트 (주신 코드 기준 COM5)

# 1. 아두이노 연결
try:
    ser = serial.Serial(ARDUINO_PORT, 9600, timeout=0.1)
    time.sleep(2)
    print("Arduino Connected")
except:
    print("Arduino Connection Failed")
    ser = None

# 2. LiDAR 연결
lidar = LidarObjectDetector(port=LIDAR_PORT)


def main():
    if not lidar.is_connected:
        print("LiDAR 연결에 실패하여 프로그램을 종료합니다.")
        return

    print("장애물 감지 주행 시작")

    # 상태 확인을 위한 빈 창 생성 (필요 없으면 제거 가능)
    cv2.namedWindow("LiDAR Status")
    status_img = np.zeros((200, 400, 3), dtype=np.uint8)

    try:
        while True:
            # 1. 장애물 인식 (LiDAR)
            # 정면(-30~30도) 80cm(800mm) 이내에 장애물 체크
            is_obstacle, count, dist = lidar.get_obstacle_status(
                min_angle=-45,
                max_angle=45,
                max_dist=1300,
                min_points=10
            )

            # 2. 제어 로직 결정
            if is_obstacle:
                # 장애물 감지 시: 조향 20, 주행 모드 0
                steering_angle = 30.0
                speed_mode = 0
                status_msg = f"OBSTACLE! ({dist:.1f}mm)"
                bg_color = (0, 0, 255)  # 빨간색 배경
            else:
                # 장애물 없을 시: 조향 0, 주행 모드 0
                steering_angle = 0.0
                speed_mode = 0
                status_msg = "CLEAR - GOING STRAIGHT"
                bg_color = (0, 255, 0)  # 초록색 배경

            # 3. 아두이노 전송 ( steering,speed_mode\n )
            if ser:
                data = f"{steering_angle},{speed_mode}\n"
                ser.write(data.encode())
                # print(f"Transmitting: {data.strip()}") # 전송 데이터 확인 로그

            # 4. 모니터링 화면 업데이트
            status_img[:] = bg_color
            cv2.putText(status_img, status_msg, (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(status_img, f"Steer: {steering_angle}", (20, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imshow("LiDAR Status", status_img)

            # 'q' 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")

    # 5. 종료 처리
    lidar.stop()
    if ser:
        ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()