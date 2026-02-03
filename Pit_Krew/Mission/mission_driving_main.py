##��ȣ��+��ֹ� �ν�
import cv2
import numpy as np
import serial
import time

# ���̺귯�� ����Ʈ (���ϸ��� ��ġ�ؾ� �մϴ�)
from obstacle_library import LidarObjectDetector
from traffic_library import Traffic_light_Detector
import AutonomousLibrary as al
import Function_Library as fl

# --- [���� �� ��Ʈ] ---
ARDUINO_PORT = 'COM6'
LIDAR_PORT = 'COM5'
STOP_RADIUS_THRESHOLD = 30
WIDTH, HEIGHT = 640, 480
LANE_WIDTH_PIXELS = 450

# ROI ���� (���� �ν� ����)
left_roi_pts = np.float32([[50, 480], [260, 480], [150, 315], [60, 270]])
right_roi_pts = np.float32([[380, 480], [580, 480], [570, 270], [490, 315]])

# --- [�ʱ�ȭ] ---
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

# ���� �ʱ�ȭ
steering_history = []
HISTORY_LIMIT = 5
prev_left_fit, prev_right_fit = None, None
last_send_time = 0
send_interval = 0.05
kernel = np.ones((5, 5), np.uint8)

# ī�޶� �ʱ� ���� (ch0: ��ȣ���, ch1: ������)
ch0, ch1 = env.initial_setting(capnum=2)


def main():
    global prev_left_fit, prev_right_fit, last_send_time

    print("���� �ý��� ���� (Cam1: ����, Cam2: ��ȣ��)")

    try:
        while True:
            # 1. ī�޶� ������ �б�
            ret, frame_traffic, ret2, frame_lane = env.camera_read(ch0, ch1)
            if not ret or not ret2: break

            frame_lane = cv2.resize(frame_lane, (WIDTH, HEIGHT))
            frame_traffic = cv2.resize(frame_traffic, (WIDTH, HEIGHT))

            # 2. LiDAR ��ֹ� Ȯ��
            is_obstacle, _, _ = lidar.get_obstacle_status(
                min_angle=-45, max_angle=45, max_dist=1300, min_points=10
            )

            # 3. ��ȣ�� �ν�
            color, _, radius = traffic_detector.object_detection(frame_traffic)

            # 4. ���� �ν� ��ó�� �� �����̵� ������
            _, green_mask = lane_detector.mask_green_floor(frame_lane)
            frame_lane_masked = lane_detector.erase_right_of_green(frame_lane, green_mask)
            combined = lane_detector.get_binary_hls(frame_lane_masked)
            cleaned = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
            refined = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

            window_img, _, left_fit, right_fit, _, valid_right = lane_detector.sliding_window(
                refined, left_roi_pts, right_roi_pts, nwindows=60, margin=50, minpix=30
            )

            # ���� ����
            left_fit, right_fit = lane_detector.predict_lane(left_fit, right_fit, lane_width=LANE_WIDTH_PIXELS)
            if lane_detector.sanity_check(left_fit, right_fit):
                prev_left_fit, prev_right_fit = left_fit, right_fit
            else:
                left_fit, right_fit = prev_left_fit, prev_right_fit

            # --- [���� ���� ����] ---
            steering_angle = 0.0
            speed_mode = 0  # 0: ����, 1: ����
            status_msg = "DRIVING"

            # �켱���� 1: ��ȣ�� (������ ����)
            if color == "RED" and radius >= STOP_RADIUS_THRESHOLD:
                speed_mode = 1
                status_msg = "STOP (RED LIGHT)"

            # �켱���� 2: ��ֹ� ȸ��
            elif is_obstacle:
                speed_mode = 0
                if valid_right:
                    steering_angle = -100.0  # �������� ����
                    status_msg = "AVOID LEFT"
                else:
                    steering_angle = 100.0  # ���������� ����
                    status_msg = "AVOID RIGHT"

            # �켱���� 3: �Ϲ� ���� ����
            elif left_fit is not None and right_fit is not None:
                raw_steer = controller.calculate_steering(left_fit, right_fit, WIDTH, HEIGHT)
                steering_history.append(raw_steer)
                if len(steering_history) > HISTORY_LIMIT: steering_history.pop(0)
                steering_angle = sum(steering_history) / len(steering_history)
                status_msg = "LANE KEEPING"

            # 5. �Ƶ��̳� ����
            curr_time = time.time()
            if ser and (curr_time - last_send_time > send_interval):
                msg = f"{steering_angle:.1f},{speed_mode}\n"
                ser.write(msg.encode())
                last_send_time = curr_time

            # 6. ����͸�
            cv2.putText(window_img, f"MSG: {status_msg}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Lane View", window_img)
            cv2.imshow("Traffic View", frame_traffic)

            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        if ser and ser.is_open:
            print("���α׷� ���� ��: ���� ���� ��� ����")
            stop_msg = "0.0,1\n"  # ���� 0, �ӵ���� 1(����)
            ser.write(stop_msg.encode())
            time.sleep(0.1)
        lidar.stop()
        if ser: ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()