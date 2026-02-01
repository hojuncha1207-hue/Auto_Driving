import cv2
import numpy as np
import lib as al
import obstacle_library as obs_lib  # 라이다 라이브러리 추가
import serial  # 아두이노 통신용 추가

# --- [아두이노 설정] ---
try:
    ser = serial.Serial('COM7', 9600, timeout=0.1) # 포트 번호 확인 필요
    print("Arduino Connected")
except Exception as e:
    print(f"Arduino Connection Error: {e}")
    ser = None

def send_to_arduino(command):
    if ser and ser.is_open:
        ser.write(command.encode())

# --- [이미지 설정] --- 영상 설정
WIDTH, HEIGHT = 640, 480
GAP_TOP, HEIGHT_TOP, X_OFFSET = 110,200,10
LANE_WIDTH_PIXELS = 220 # 차선 폭

# --- [사용자 지정 임의 ROI 좌표] ---
left_roi_pts = np.float32([[115, 480], [295, 480], [292, 455], [130, 395]])
right_roi_pts = np.float32([[355, 480], [525, 480], [510, 395], [358, 455]])
nwindows=60

detector = al.LaneDetector(WIDTH, HEIGHT)
controller = al.PurePursuitController()
lidar_detector = obs_lib.LidarObjectDetector(port='COM5') # 라이다 객체 생성

cap = cv2.VideoCapture("curv.mp4")
kernel = np.ones((5, 5), np.uint8)
prev_left_fit, prev_right_fit = None, None

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("영상이 끝났거나 파일을 찾을 수 없습니다.")
        break
    img = cv2.resize(frame, (WIDTH, HEIGHT))

    # --- [라이다 장애물 감지 추가] ---
    is_obs, obs_count, obs_dist = lidar_detector.get_obstacle_status(max_dist=400)

    # 0단계: 초록색 바닥 마스킹
    _, green_mask = detector.mask_green_floor(img)

    # 3. 초록색 기준 오른쪽 영역 삭제
    clean_img = detector.erase_right_of_green(img, green_mask)

    # 4. 전처리
    orbev_img, white_lane, sx_binary, combined = detector.get_binary_hls(clean_img)

    # 5. 모폴로지 cleanup
    cleaned = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

    block_refined = detector.block_filter(cleaned, nwindows=nwindows, min_w=7, max_w=60)
    refined = detector.filter_by_shape(block_refined)

    # 2. 하단 삼각형 여백 채우기
    last=detector.fill_bev_dead_zones(refined, y_start=450)
    refined_extended = detector.extend_pixels_side_only(last, y_start=400, center_gap=180)

    # 6. 슬라이딩 윈도우 적용
    window_img, offset, left_fit, right_fit, valid_left, valid_right = detector.sliding_window(
        refined_extended, left_roi_pts, right_roi_pts, nwindows=nwindows, margin=50, minpix=30)

    # 차선 타입 판별
    left_type = detector.detect_line_type(valid_left, min_ratio=0.6)
    right_type = detector.detect_line_type(valid_right, min_ratio=0.6)

    # --- [아두이노 신호 전송 로직 추가] ---
    if is_obs:
        if right_type == "Solid":
            print(f"장애물({obs_dist:.0f}mm): Left")
            send_to_arduino('L')
        elif left_type == "Solid":
            print(f"장애물({obs_dist:.0f}mm): Right")
            send_to_arduino('R')
        else:
            print(f"🚨 장애물({obs_dist:.0f}mm): Stop")
            send_to_arduino('S')
    else:
        send_to_arduino('G') # 정상 주행(Go) 신호
        print(f"✅ 주행 중... (장애물 없음)", end='\r')

    # 3. 화면 텍스트 그리기 (기존 코드 유지)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    thickness = 2
    text_color = (0, 255, 255)
    cv2.putText(window_img, f"L: {left_type}", (20, 80), font, font_scale, text_color, thickness)
    right_text = f"R: {right_type}"
    text_size, _ = cv2.getTextSize(right_text, font, font_scale, thickness)
    text_x = WIDTH - text_size[0] - 20
    cv2.putText(window_img, right_text, (text_x, 80), font, font_scale, text_color, thickness)

    cv2.polylines(window_img, [np.int32(left_roi_pts)], True, (0, 0, 255), 3)
    cv2.polylines(window_img, [np.int32(right_roi_pts)], True, (255, 0, 0), 3)

    # [4] 예측 로직
    left_fit, right_fit = detector.predict_lane(left_fit, right_fit, lane_width=LANE_WIDTH_PIXELS)

    # [5] 검증
    if detector.sanity_check(left_fit, right_fit):
        prev_left_fit, prev_right_fit = left_fit, right_fit
    else:
        left_fit, right_fit = prev_left_fit, prev_right_fit

    # [6] 조향각 시각화
    if left_fit is not None and right_fit is not None:
        steering = controller.calculate_steering(left_fit, right_fit, WIDTH, HEIGHT)
        cv2.putText(window_img, f"Steer: {steering:.2f} deg", (30, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # 결과 출력
    cv2.imshow("6. Sliding Windows", window_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
lidar_detector.stop() # 라이다 정지 추가
if ser: ser.close() # 시리얼 닫기 추가
cv2.destroyAllWindows()