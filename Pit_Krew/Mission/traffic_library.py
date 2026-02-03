import cv2
import numpy as np

RED, GREEN = (0, 1)
COLOR_NAMES = ("RED", "GREEN")


class Traffic_light_Detector:
    def hsv_conversion(self, img):
        if img is None or img.size == 0: return None
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)

    def color_detection(self, img, color_id):
        hsv = self.hsv_conversion(img)
        if hsv is None: return None, None

        # 실내 환경을 고려해 채도(S)와 명도(V) 범위를 50~255로 설정
        if color_id == RED:
            mask1 = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([160, 50, 50]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)
        elif color_id == GREEN:
            mask = cv2.inRange(hsv, np.array([35, 50, 50]), np.array([95, 255, 255]))
        else:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        mask = cv2.GaussianBlur(mask, (7, 7), 0)
        return mask

    def object_detection(self, img):
        detected_color = "NONE"
        detected_radius = 0
        debug_mask = np.zeros(img.shape[:2], dtype=np.uint8)
        center_y=100;

        for cid in [RED, GREEN]:
            mask = self.color_detection(img, cid)
            if mask is None: continue

            # 원형 검출 (HoughCircles)
            circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                       param1=100, param2=20, minRadius=10, maxRadius=200)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                # 가장 먼저 발견된 원의 정보를 가져옴
                best_circle = circles[0, 0]
                detected_color = COLOR_NAMES[cid]
                detected_radius = int(best_circle[2])  # 반지름 저장
                center = (int(best_circle[0]), int(best_circle[1]))  # (x, y)
                center_y=center[1]
                debug_mask = mask
                break

            # 원이 없어도 색상이 감지되면 디버깅용으로 마스크 저장
            if np.any(mask > 0): debug_mask = mask

        return detected_color, debug_mask, detected_radius,center_y