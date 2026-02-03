# -*- coding: utf-8 -*-
import cv2
import numpy as np
import serial
import time

# 라이브러리 임포트
import AutonomousLibrary as al
import Function_Library as fl

# --- [설정 및 상수] ---
WIDTH, HEIGHT = 640, 480
LANE_WIDTH_PIXELS = 450
ARDUINO_PORT = 'COM3'  # 아두이노 포트 번호 (장치관리자 확인 필수)
BAUDRATE = 9600  # 아두이노 코드와 속도 일치

# ROI 좌표 설정
left_roi_pts = np.float32([[50, 480], [260, 480], [150, 315], [60, 270]])
right_roi_pts = np.float32([[380, 480], [580, 480], [570, 270], [490, 315]])


# --- [유틸리티 함수] ---
def apply_roi(frame, x_start, x_end, y_start, y_end):
    """설정된 범위 외에는 전부 검은색으로 처리"""
    mask = np.zeros_like(frame)
    # 범위 예외 처리
    h, w = frame.shape[:2]
    x_start, x_end = max(0, x_start), min(w, x_end)
    y_start, y_end = max(0, y_start), min(h, y_end)

    mask[y_start:y_end, x_start:x_end] = frame[y_start:y_end, x_start:x_end]
    return mask


# --- [기능 확장 클래스] ---
class SmartCamera(fl.libCAMERA):
    def object_detection_with_pos(self, img, sample=0, mode="circle", print_enable=False):
        """신호등 색상과 Y좌표를 함께 반환"""
        result_color = "NONE"
        center_y = -1
        replica = img.copy()

        for color in (fl.RED, fl.YELLOW, fl.GREEN):
            extract = self.color_filtering(img, roi=color, print_enable=False)
            gray = self.gray_conversion(extract)
            circles = self.hough_transform(gray, mode=mode)

            if circles is not None:
                for circle in circles[0]:
                    center = (int(circle[0]), int(circle[1]))
                    radius = int(circle[2])
                    count = 0

                    hsv_img = self.hsv_conversion(img)
                    h, s, v = cv2.split(hsv_img)

                    for res in range(sample):
                        x = int(center[1] - sample / 2)
                        y = int(center[0] - sample / 2)
                        if x < 0 or x >= h.shape[0] or y < 0 or y >= h.shape[1]: continue

                        s_cond = s[x][y] > fl.SATURATION
                        if color == fl.RED:
                            h_cond = (h[x][y] < fl.HUE_THRESHOLD[color][0]) | (h[x][y] > fl.HUE_THRESHOLD[color][1])
                        else:
                            h_cond = (h[x][y] > fl.HUE_THRESHOLD[color][0]) & (h[x][y] < fl.HUE_THRESHOLD[color][1])

                        if h_cond and s_cond: count += 1

                    if count > sample / 2:
                        result_color = fl.COLOR[color]
                        center_y = center[1]
                        cv2.circle(replica, center, radius, (0, 0, 255), 2)
                        cv2.putText(replica, f"{result_color} y:{center_y}", (center[0], center[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        break

        if print_enable:
            cv2.imshow("Traffic Light Debug", replica)

        return result_color, center_y


# --- [시스템 초기화] ---
try:
    ser = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=0)
    time.sleep(2.0)
    print(f"Arduino Connected on {ARDUINO_PORT}")
except Exception as e:
    print(f"Serial Error: {e}")
    ser = None

lane_detector = al.LaneDetector(WIDTH, HEIGHT)
controller = al.PurePursuitController()
det = al.LaneDetector()

# 카메라 설정
env = SmartCamera()
# [중요] 카메라 포트 번호 확인 (차선:1, 신호등:2 등 환경에 맞게)
ch_lane, ch_traffic = env.initial_setting(cam0port=1, cam1port=2, capnum=2)

# 전역 변수
steering_history = []
HISTORY_LIMIT = 5
prev_left_fit, prev_right_fit = None, None
last_send_time = 0
send_interval = 0.05
kernel = np.ones((5, 5), np.uint8)


def main():
    global prev_left_fit, prev_right_fit, last_send_time, steering_history

    print("=== 자율주행 시스템 가동 ===")
    print("설정: 0=주행, 1=정지")

    # 초기 차선 값 (에러 방지용)
    prev_left_fit = np.array([0, 0, 150])
    prev_right_fit = np.array([0, 0, 450])

    # 상태 변수는 루프 밖에서 선언
    waiting_for_green = False

    try:
        while True:
            # 1. 카메라 데이터 읽기
            ret_lane, frame_lane, ret_traffic, frame_traffic = env.camera_read(ch_lane, ch_traffic)
            if not ret_lane or not ret_traffic:
                print("카메라 영상 읽기 실패")
                break

            frame_lane = cv2.resize(frame_lane, (WIDTH, HEIGHT))
            frame_traffic = cv2.resize(frame_traffic, (WIDTH, HEIGHT))

            # ------------------------------------------------
            # [TASK A] 신호등 인식 및 상태 제어
            # ------------------------------------------------
            roi_traffic = apply_roi(frame_traffic, x_start=0, x_end=WIDTH, y_start=0, y_end=150)
            tl_color, tl_y_pos = env.object_detection_with_pos(roi_traffic, sample=5, print_enable=True)

            # [수정됨] 기본값: 0 (주행)
            drive_mode = 0
            status_msg = "DRIVING"

            # 신호등 상태 머신 로직
            if waiting_for_green == False:
                # (1) 주행 중일 때: 빨간불 감시
                if tl_color == "RED":
                    if tl_y_pos < 230:  # 상단에 빨간불이 뜨면
                        waiting_for_green = True  # 대기 모드 진입
                        drive_mode = 1  # [수정됨] 1 (정지)
                        print(f"🚨 [STOP] 빨간불 감지 (Y={tl_y_pos}) -> 정지")
            else:
                # (2) 대기 중일 때: 초록불 감시
                if tl_color == "GREEN":
                    waiting_for_green = False  # 대기 모드 해제
                    drive_mode = 0  # [수정됨] 0 (주행)
                    print("🟢 [GO] 초록불 감지 -> 출발")
                else:
                    drive_mode = 1  # [수정됨] 1 (계속 정지)
                    status_msg = "WAITING FOR GREEN"
                    print("⏳ 신호 대기 중... (정지 유지)")

            # ------------------------------------------------
            # [TASK B] 차선 인식 (항상 실행)
            # ------------------------------------------------
            # 2. 이진화
            _, green_mask = lane_detector.mask_green_floor(frame_lane)
            frame_lane_masked = lane_detector.erase_right_of_green(frame_lane, green_mask)
            combined = lane_detector.get_binary_hls(frame_lane_masked)

            # 3. 모폴로지
            cleaned = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
            refined = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

            # 4. 필터링 및 윈도우 탐색
            block_refined = lane_detector.block_filter(refined, nwindows=60, min_w=10, max_w=33)
            final_img = lane_detector.filter_by_shape(block_refined)

            window_img, _, left_fit, right_fit, _, _ = lane_detector.sliding_window(
                final_img, left_roi_pts, right_roi_pts, nwindows=60, margin=50, minpix=30
            )

            # 5. 차선 예측 및 스무딩
            left_fit, right_fit = lane_detector.predict_lane(left_fit, right_fit, lane_width=LANE_WIDTH_PIXELS)

            if lane_detector.sanity_check(left_fit, right_fit):
                prev_left_fit, prev_right_fit = left_fit, right_fit
            else:
                left_fit, right_fit = prev_left_fit, prev_right_fit

            # 6. 조향각 계산
            steering_angle = 0.0
            if left_fit is not None and right_fit is not None:
                raw_steer = controller.calculate_steering(left_fit, right_fit, WIDTH, HEIGHT)

                # 값 튀는 현상 방지 (스무딩)
                if not steering_history:
                    steering_angle = raw_steer
                    steering_history.append(steering_angle)
                else:
                    avg_steering = sum(steering_history) / len(steering_history)
                    if abs(raw_steer - avg_steering) <= 150:
                        steering_angle = raw_steer
                        steering_history.append(steering_angle)
                        steering_history.pop(0)
                    else:
                        steering_angle = avg_steering  # 이상치 발생 시 평균값 사용

            # ------------------------------------------------
            # [TASK C] 아두이노 전송 (디버깅 모드)
            # ------------------------------------------------
            curr_time = time.time()

            # (중요) 아두이노 연결이 안 되어 있어도 로그는 찍히게 수정
            if (curr_time - last_send_time > send_interval):

                # 정지 상태(1)일 때는 조향각 0으로 보냄
                if drive_mode == 1:
                    send_steer = 0.0
                else:
                    send_steer = steering_angle

                # [전송 포맷] "조향각,모드\n"
                msg = f"{send_steer:.1f},{drive_mode}\n"

                # --- [수정] 보내는 값을 무조건 화면에 출력합니다 ---
                if drive_mode == 0:
                    # 0 = 주행 (초록색 메시지)
                    print(f"🚀 [주행신호 전송] {msg.strip()} (Steer: {send_steer:.1f})")
                else:
                    # 1 = 정지 (빨간색 메시지)
                    print(f"⛔ [정지신호 전송] {msg.strip()} (RED LIGHT or WAIT)")
                # --------------------------------------------------

                if ser and ser.is_open:
                    ser.write(msg.encode())

                    if ser.in_waiting > 0:
                        ser.read(ser.in_waiting)

                last_send_time = curr_time

            # ------------------------------------------------
            # [TASK D] 시각화
            # ------------------------------------------------
            cv2.polylines(window_img, [np.int32(left_roi_pts)], True, (0, 0, 255), 2)
            cv2.polylines(window_img, [np.int32(right_roi_pts)], True, (255, 0, 0), 2)
            cv2.putText(window_img, f"Mode: {drive_mode} ({status_msg})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(window_img, f"Steer: {steering_angle:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Lane Tracking (Cam1)", window_img)
            # cv2.imshow("Traffic (Cam2)", frame_traffic) # 필요시 주석 해제

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Main Loop Error: {e}")

    finally:
        # 종료 시 안전하게 정지 명령 전송 (1: 정지)
        if ser and ser.is_open:
            ser.write("0.0,1\n".encode())
            ser.close()
            print("System Shutdown - Motors Stopped")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()