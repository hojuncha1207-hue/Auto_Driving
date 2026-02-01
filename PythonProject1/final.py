import cv2
import numpy as np
import lib as al
import traffic_library as al_traffic  # 신호등 라이브러리
import obstacle_library as obs_lib  # 라이다 라이브러리
import serial  # 아두이노 통신


def main():
    # --- [이미지 및 주행 설정] ---
    WIDTH, HEIGHT = 640, 480
    LANE_WIDTH_PIXELS = 320
    STOP_LINE_LIMIT = 420  # 정지선 인식 기준선
    nwindows = 60

    # --- [아두이노 설정] ---
    try:
        ser = serial.Serial('COM7', 9600, timeout=0.1)
        print("Arduino Connected")
    except Exception as e:
        print(f"Arduino Connection Error: {e}")
        ser = None

    def send_to_arduino(command):
        if ser and ser.is_open:
            ser.write(command.encode())

    # --- [ROI 설정] ---
    left_roi_pts = np.float32([[115, 480], [295, 480], [292, 455], [130, 395]])
    right_roi_pts = np.float32([[355, 480], [525, 480], [510, 395], [358, 455]])

    # 객체 생성
    detector = al.LaneDetector(WIDTH, HEIGHT)
    controller = al.PurePursuitController()
    traffic_env = al_traffic.Traffic_light_Detector()
    lidar_detector = obs_lib.LidarObjectDetector(port='COM5')

    cap = cv2.VideoCapture("curv.mp4")
    kernel = np.ones((5, 5), np.uint8)
    prev_left_fit, prev_right_fit = None, None

    print("시스템 시작")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break

        img = cv2.resize(frame, (WIDTH, HEIGHT))
        display_img = img.copy()

        # ------------------------------------------------------
        # 1. 라이다 및 신호등 인식 (Sensing)
        # ------------------------------------------------------
        # [라이다] 장애물 감지
        is_obs, obs_count, obs_dist = lidar_detector.get_obstacle_status(max_dist=400)

        # [신호등] ROI 및 상태 감지
        tl_roi_x1, tl_roi_y1 = int(WIDTH * 0.2), 0
        tl_roi_x2, tl_roi_y2 = int(WIDTH * 0.6), int(HEIGHT * 0.4)
        tl_roi = img[tl_roi_y1:tl_roi_y2, tl_roi_x1:tl_roi_x2]
        traffic_state, traffic_vis = traffic_env.object_detection(tl_roi)
        stop_y = traffic_env.get_stop_line(img)

        # ------------------------------------------------------
        # 2. 차선 인식 전처리 (Lane Processing)
        # ------------------------------------------------------
        _, green_mask = detector.mask_green_floor(img)
        clean_img = detector.erase_right_of_green(img, green_mask)
        orbev_img, white_lane, _, combined = detector.get_binary_hls(clean_img)

        cleaned = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)

        block_refined = detector.block_filter(cleaned, nwindows=nwindows)
        refined = detector.filter_by_shape(block_refined)
        last = detector.fill_bev_dead_zones(refined, y_start=450)
        refined_extended = detector.extend_pixels_side_only(last, y_start=400, center_gap=180)

        # 슬라이딩 윈도우
        window_img, offset, left_fit, right_fit, valid_left, valid_right = detector.sliding_window(
            refined_extended, left_roi_pts, right_roi_pts, nwindows=nwindows)

        left_type = detector.detect_line_type(valid_left, min_ratio=0.6)
        right_type = detector.detect_line_type(valid_right, min_ratio=0.6)

        # ------------------------------------------------------
        # 3. 통합 주행 판단 및 신호 전송 (Decision & Control)
        # ------------------------------------------------------
        drive_msg = "DRIVING"

        # [우선순위 1] 라이다 장애물 감지 시 회피
        if is_obs:
            if right_type == "Solid":
                drive_msg = "AVOID LEFT"
                send_to_arduino('L')
            elif left_type == "Solid":
                drive_msg = "AVOID RIGHT"
                send_to_arduino('R')
            else:
                drive_msg = "OBS STOP"
                send_to_arduino('S')

        # [우선순위 2] 신호등이 빨간불이고 정지선이 가까울 때 정지
        elif traffic_state == "RED" and stop_y and stop_y > STOP_LINE_LIMIT:
            drive_msg = "TRAFFIC STOP"
            send_to_arduino('S')

        # [우선순위 3] 정상 주행
        else:
            send_to_arduino('G')

        # ------------------------------------------------------
        # 4. 조향 및 시각화 (Visualization)
        # ------------------------------------------------------
        left_fit, right_fit = detector.predict_lane(left_fit, right_fit, lane_width=LANE_WIDTH_PIXELS)
        if detector.sanity_check(left_fit, right_fit):
            prev_left_fit, prev_right_fit = left_fit, right_fit
        else:
            left_fit, right_fit = prev_left_fit, prev_right_fit

        steering = 0
        if left_fit is not None and right_fit is not None:
            steering = controller.calculate_steering(left_fit, right_fit, WIDTH, HEIGHT)

        # 결과 화면 그리기
        cv2.rectangle(display_img, (tl_roi_x1, tl_roi_y1), (tl_roi_x2, tl_roi_y2), (255, 0, 0), 2)
        cv2.putText(display_img, f"LIGHT: {traffic_state} | MODE: {drive_msg}", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if stop_y:
            cv2.line(display_img, (0, stop_y), (WIDTH, stop_y), (0, 0, 255), 3)

        cv2.putText(window_img, f"Steer: {steering:.2f} deg", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.polylines(window_img, [np.int32(left_roi_pts)], True, (0, 0, 255), 2)
        cv2.polylines(window_img, [np.int32(right_roi_pts)], True, (255, 0, 0), 2)

        # 창 출력
        cv2.imshow("1. Color & Traffic", display_img)
        cv2.imshow("6. Sliding Windows", window_img)
        cv2.imshow("Traffic Filter", traffic_vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    lidar_detector.stop()
    if ser: ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()