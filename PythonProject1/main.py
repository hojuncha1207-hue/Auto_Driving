import cv2
import numpy as np
import serial
import time

# 라이브러리 임포트 (파일명이 일치해야 합니다)
from obstacle_library import LidarObjectDetector
from traffic_library import Traffic_light_Detector
import AutonomousLibrary as al
import Function_Library as fl

# --- [설정 및 포트] ---
ARDUINO_PORT = 'COM6'
LIDAR_PORT = 'COM5'
STOP_RADIUS_THRESHOLD = 30
WIDTH, HEIGHT = 640, 480
LANE_WIDTH_PIXELS = 450

# ROI 설정 (차선 인식 영역)
left_roi_pts = np.float32([[50, 480], [260, 480], [150, 315], [60, 270]])
right_roi_pts = np.float32([[380, 480], [580, 480], [570, 270], [490, 315]])

# --- [초기화] ---
try:
    ser = serial.Serial(ARDUINO_PORT, 9600, timeout=0)
    time.sleep(2.0)
    print("Arduino Connected!")
except Exception as e:
    print(f"Serial Error: {e}")
    ser = None

lidar = LidarObjectDetector(port=LIDAR_PORT)
traffic_detector = Traffic_light_Detector()
lane_detector = al.LaneDetector(WIDTH, HEIGHT)
controller = al.PurePursuitController()
env = fl.libCAMERA()

# 변수 초기화
steering_history = []
HISTORY_LIMIT = 5
prev_left_fit, prev_right_fit = None, None
last_send_time = 0
send_interval = 0.05
kernel = np.ones((5, 5), np.uint8)

# 카메라 초기 설정 (ch0: 신호등용, ch1: 차선용)
ch0, ch1 = env.initial_setting(capnum=2)


def main():
    global prev_left_fit, prev_right_fit, last_send_time

    print("통합 시스템 시작 (Cam1: 차선, Cam2: 신호등)")

    try:
        while True:
            # 1. 카메라 데이터 읽기
            ret, frame_traffic, ret2, frame_lane = env.camera_read(ch0, ch1)
            if not ret or not ret2: break

            frame_lane = cv2.resize(frame_lane, (WIDTH, HEIGHT))
            frame_traffic = cv2.resize(frame_traffic, (WIDTH, HEIGHT))

            # 2. LiDAR 장애물 확인
            is_obstacle, _, _ = lidar.get_obstacle_status(
                min_angle=-45, max_angle=45, max_dist=1300, min_points=10
            )

            # 3. 신호등 인식
            color, _, radius = traffic_detector.object_detection(frame_traffic)

            # 4. 차선 인식 전처리 및 슬라이딩 윈도우
            _, green_mask = lane_detector.mask_green_floor(frame_lane)
            frame_lane_masked = lane_detector.erase_right_of_green(frame_lane, green_mask)
            combined = lane_detector.get_binary_hls(frame_lane_masked)
            cleaned = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
            refined = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

            window_img, _, left_fit, right_fit, _, valid_right = lane_detector.sliding_window(
                refined, left_roi_pts, right_roi_pts, nwindows=60, margin=50, minpix=30
            )

            # 차선 보정
            left_fit, right_fit = lane_detector.predict_lane(left_fit, right_fit, lane_width=LANE_WIDTH_PIXELS)
            if lane_detector.sanity_check(left_fit, right_fit):
                prev_left_fit, prev_right_fit = left_fit, right_fit
            else:
                left_fit, right_fit = prev_left_fit, prev_right_fit

            # --- [제어 로직 결정] ---
            steering_angle = 0.0
            speed_mode = 0  # 0: 주행, 1: 정지
            status_msg = "DRIVING"

            # 우선순위 1: 신호등 (빨간불 정지)
            if color == "RED" and radius >= STOP_RADIUS_THRESHOLD:
                speed_mode = 1
                status_msg = "STOP (RED LIGHT)"

            # 우선순위 2: 장애물 회피
            elif is_obstacle:
                speed_mode = 0
                if valid_right:
                    steering_angle = -100.0  # 왼쪽으로 꺾기
                    status_msg = "AVOID LEFT"
                else:
                    steering_angle = 100.0  # 오른쪽으로 꺾기
                    status_msg = "AVOID RIGHT"

            # 우선순위 3: 일반 차선 주행
            elif left_fit is not None and right_fit is not None:
                raw_steer = controller.calculate_steering(left_fit, right_fit, WIDTH, HEIGHT)
                steering_history.append(raw_steer)
                if len(steering_history) > HISTORY_LIMIT: steering_history.pop(0)
                steering_angle = sum(steering_history) / len(steering_history)
                status_msg = "LANE KEEPING"

            # 5. 아두이노 전송
            curr_time = time.time()
            if ser and (curr_time - last_send_time > send_interval):
                msg = f"{steering_angle:.1f},{speed_mode}\n"
                ser.write(msg.encode())
                last_send_time = curr_time

            # 6. 모니터링
            cv2.putText(window_img, f"MSG: {status_msg}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Lane View", window_img)
            cv2.imshow("Traffic View", frame_traffic)

            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        if ser and ser.is_open:
            print("프로그램 종료 중: 모터 정지 명령 전송")
            stop_msg = "0.0,1\n"  # 조향 0, 속도모드 1(정지)
            ser.write(stop_msg.encode())
            time.sleep(0.1)
        lidar.stop()
        if ser: ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()