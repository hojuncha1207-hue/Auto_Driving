import cv2
import numpy as np
import AutonomousLibrary as al
import serial
import time
import Function_Library as fl


# --- [시리얼 설정] ---
try:
 # 아두이노가 연결된 포트로 수정 (COM12 등)
 ser = serial.Serial('COM12', 9600, timeout=0)
 time.sleep(2.0)  # 아두이노 리셋 대기
 print("Arduino Connected!")
except Exception as e:
 print(f"Serial Error: {e}")
 ser = None


# --- [이미지 설정] ---
WIDTH, HEIGHT = 640, 480
LANE_WIDTH_PIXELS = 450


# --- [사용자 지정 임의 ROI 좌표] ---
# --- [사용자 지정 임의 ROI 좌표] ---# 좌측 하단, 우측 하단, 우측 상단, 좌측 상단 순서
left_roi_pts = np.float32([[50, 480], [260, 480], [150, 315], [60, 270]])
right_roi_pts = np.float32([[380, 480], [580, 480], [570, 270], [490, 315]])






nwindows = 60


# --- [제어 변수 초기화] ---
steering_history = []
HISTORY_LIMIT = 5
STEER_THRESHOLD = 15


detector = al.LaneDetector(WIDTH, HEIGHT)
controller = al.PurePursuitController()


prev_left_fit, prev_right_fit = None, None
last_send_time = 0
send_interval = 0.05  # 0.1초는 너무 느릴 수 있어 0.05초로 단축 추천
last_cmd = -999  # 초기값 변경


# 커널 설정
kernel = np.ones((5, 5), np.uint8)


EPOCH = 500000
env = fl.libCAMERA()
det=al.LaneDetector()


GAP_TOP, HEIGHT_TOP, X_OFFSET = 190, 250,50
# Camera Initial Setting
ch0,ch1 = env.initial_setting(capnum=2)




for i in range(EPOCH):
 ret, frame0, ret2, frame1 = env.camera_read(ch0, ch1)
 if not ret:
     print("영상 종료")
     break


 frame0 = cv2.resize(frame0, (WIDTH, HEIGHT))
 frame1 = cv2.resize(frame1, (WIDTH, HEIGHT))
 # --- [이미지 전처리] ---
 # 2. 이진화 (HLS 등)
 _, green_mask = det.mask_green_floor(frame1)
 frame1_masked = det.erase_right_of_green(frame1, green_mask)
 combined = detector.get_binary_hls(frame1_masked)


 # 3. 모폴로지 연산
 cleaned = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
 cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)


 # 4. 필터링 및 확장 이거도 지워도 뒬듯 아래로 전부?
 block_refined = detector.block_filter(cleaned, nwindows=nwindows, min_w=10, max_w=33)
 refined = detector.filter_by_shape(block_refined)


 # 5. 슬라이딩 윈도우
 window_img, offset, left_fit, right_fit, valid_left, valid_right = detector.sliding_window(
     refined, left_roi_pts, right_roi_pts,
     nwindows=nwindows, margin=50, minpix=30
 )


 # 6. 차선 예측 및 보정
 left_fit, right_fit = detector.predict_lane(left_fit, right_fit, lane_width=LANE_WIDTH_PIXELS)


 # --- [핵심 수정 구간: 데이터 검증 및 유지] ---
 if detector.sanity_check(left_fit, right_fit):
     prev_left_fit, prev_right_fit = left_fit, right_fit
 else:
     # 인식이 안 되면 이전 값 사용
     left_fit, right_fit = prev_left_fit, prev_right_fit


 # --- [조향각 계산 및 전송 로직] ---
 # 중요: 이 부분은 if/else 밖으로 나와야 매번 실행됩니다!
 steering = 0
 if left_fit is not None and right_fit is not None:
     # 1. 원본 조향각 계산
     raw_steering = controller.calculate_steering(left_fit, right_fit, WIDTH, HEIGHT)


     # 2. 값 튀는 현상 방지 (스무딩)
     if len(steering_history) < HISTORY_LIMIT:
         steering = raw_steering
         steering_history.append(steering)
     else:
         avg_steering = sum(steering_history) / len(steering_history)


         if abs(raw_steering - avg_steering) <= 150:
             # 정상 범위: 값 채택 및 히스토리 업데이트
             steering = raw_steering
             steering_history.append(steering)
             steering_history.pop(0)  # 오래된 값 삭제
         else:
             # 이상치 발생: 평균값 사용 (안전)
             steering = avg_steering
             # 이상치라도 히스토리에 넣을지 말지는 선택 사항 (여기선 안전하게 유지)


     # 3. 시각화
     # cv2.putText(window_img, f"Steer: {steering:.2f} deg", (30, 50),
     #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
     print(f"Steer: {steering:.2f}deg")


     # 4. 아두이노 전송 (타이머 적용)
     current_time = time.time()


     # 방향 판단 (단순 로깅용, 실제 전송은 steering 각도)
     if steering > 4:
         direction = 2  # Right
     elif steering < -4:
         direction = 1  # Left
     else:
         direction = 0  # Straight


     if ser and ser.is_open:
         # 일정 시간(send_interval)이 지났거나, 방향이 급격히 바뀌었을 때 전송
         if (current_time - last_send_time > send_interval):


             # 아두이노 코드는 parseFloat를 쓰므로 "각도\n" 형태로 전송
             # 예: "12.5\n"
             msg = f"{steering:.1f}\n"
             ser.write(msg.encode())


             last_send_time = current_time


             print(msg)
             # 시리얼 버퍼 비우기 (지연 방지)
             if ser.in_waiting > 0:
                 ser.read(ser.in_waiting)


             print(f"Sent: {msg.strip()}")  # 디버깅용 출력


 # --- [결과 출력] ---//몇번 보고 지우기
 # cv2.imshow("frame0", frame0)
 # cv2.imshow("frame1", frame1)
 #cv2.imshow("Processed", refined)
# 왼쪽 ROI: 빨간색 (BGR: 0, 0, 255), 두께 3
# cv2.polylines(window_img, [np.int32(left_roi_pts)], True, (0, 0, 255), 3)
# 오른쪽 ROI: 파란색 (BGR: 255, 0, 0), 두께 3
#     cv2.polylines(window_img, [np.int32(right_roi_pts)], True, (255, 0, 0), 3)
 cv2.polylines(window_img, [np.int32(left_roi_pts)], isClosed=True, color=(0, 0, 255), thickness=2)
 # 우측 ROI: 파란색 (BGR: 255, 0, 0), 두께 2
 cv2.polylines(window_img, [np.int32(right_roi_pts)], isClosed=True, color=(255, 0, 0), thickness=2)
 cv2.imshow("combined", combined)
 cv2.imshow("Result", window_img)


 if cv2.waitKey(1) & 0xFF == ord('q'):
     break




cv2.destroyAllWindows()
if ser:
 ser.close()







