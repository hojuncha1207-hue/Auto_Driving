import numpy as np
from rplidar import RPLidar
import math

MIN_DISTANCE = 200
MAX_DISTANCE = 6000

SCAN_TYPE = "normal"
SAMPLE_RATE = 10
MAX_BUFFER_SIZE = 3000

ANGLE_MIN = 310
ANGLE_MAX = 360
MOVE_EPS_MM = 1.0  # 1mm 이내 변화는 "Mid"

class libLIDAR(object):

    def __init__(self, port):
        self.rpm = 0
        self.lidar = RPLidar(port)
        self.scan = []
        self.is_connected = False
        self.lidar = None
        self.scan_generator = None

        try:
            self.lidar = RPLidar(port, baudrate=115200, timeout=0.5)
            self.lidar.connect()
            self.lidar.start_motor()
            self.scan_generator = self.lidar.iter_scans(max_buf_meas=500)
            self.is_connected = True
            print(f"LiDAR connected on {port}")
        except Exception as e:
            print(f"LiDAR Connection Error: {e}")

    def init(self):
        info = self.lidar.get_info()
        print(info)

    def getState(self):
        health = self.lidar.get_health()
        print(health)

    def scanning(self):
        scan_list = []
        iterator = self.lidar.iter_measures(SCAN_TYPE, MAX_BUFFER_SIZE)
        for new_scan, quality, angle, distance in iterator:
           if new_scan:
               if len(scan_list) > SAMPLE_RATE:
                   np_data = np.array(list(scan_list))
                   yield np_data[:, 1:]
               scan_list = []
           if distance > MIN_DISTANCE:
               scan_list.append((quality, angle, distance))

    def stop(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def setRPM(self, rpm):
        self.lidar.motor_speed = rpm

    def getRPM(self):
        return self.lidar.motor_speed

    def getAngleRange(self, scan, minAngle, maxAngle):
        data = np.array(scan)
        condition = np.where((data[:, 0] < maxAngle) & (data[:, 0] > minAngle))
        return data[condition]

    def getDistanceRange(self, scan, minDist, maxDist):
        data = np.array(scan)
        condition = np.where((data[:, 1] < maxDist) & (data[:, 1] > minDist))
        return data[condition]

    def getAngleDistanceRange(self, scan, minAngle, maxAngle, minDist, maxDist):
        data = np.array(scan)
        condition = np.where((data[:, 0] < maxAngle) & (data[:, 0] > minAngle) & (data[:, 1] < maxDist) & (data[:, 1] > minDist))
        return data[condition]

    def extract_angle_distance(self, scan_np):
        """
        scan으로부터 각도(angle)과 거리(dist) 값만 받기
        scanning은 (품질, 각도, 거리) 리스트와 datatype이 다름.
        """
        scan = np.asarray(scan_np)
        angle = scan[:, 0].astype(np.float32)
        dist = scan[:, 1].astype(np.float32)
        return angle, dist

    def select_front_sector(self, angle, dist, half_width=30.0, min_dist=1.0):
        """
        0도를 전방으로 보고 ±half_width 범위 선택
        예: half_width=30 → [330~360) U [0~30]
        """
        angle = np.asarray(angle)
        dist = np.asarray(dist)

        m = (
                ((angle >= 360 - half_width) | (angle <= half_width))
                & (dist >= min_dist)
        )

        a = angle[m]
        d = dist[m]

        if a.size == 0:
            return a, d

        idx = np.argsort(a)
        return a[idx], d[idx]

    def select_and_sort_sector(self, angle, dist, ang_min=0, ang_max=359, min_dist=1.0):
        """
        전방 각도와 유효 거리 설정 후 angle 오름차순 정렬
        """
        angle = np.asarray(angle)
        dist = np.asarray(dist)

        mask = (angle >= ang_min) & (angle <= ang_max) & (dist >= min_dist)
        a = angle[mask]
        d = dist[mask]

        if a.size == 0:
            return a, d  # 빈 배열

        idx = np.argsort(a)
        return a[idx], d[idx]

    def median_smooth_1d(self, x, k=5):
        """
        dist를 median filter로 노이즈 줄이기. 튀는 값 제거
        갑자기 거리가 0 또는 99999로 튀는 상황에서 "물체 끝"으로 인식하지 않기 위한 전처리 과정
        k: 홀수 권장 (3,5,7...)
        """
        x = np.asarray(x, dtype=np.float32)
        if x.size == 0:
            return x
        if k <= 1:
            return x.copy()
        if k % 2 == 0:
            raise ValueError("k should be odd for a symmetric median filter")

        pad = k // 2
        y = x.copy()

        # 양 끝은 그대로 두고 중앙만 median 적용
        for i in range(pad, x.size - pad):
            y[i] = np.median(x[i - pad:i + pad + 1])
        return y

    def detect_jump_edges(self, dist, jump_thresh=400.0):
        """
        갑자기 거리가 튀는 부분을 물체 끝으로 인식하기

        dist: 정렬 + median 된 거리
        jump_thresh: 급변 임계값 (mm 기준이면 200~800 등 환경튜닝)
        return: edges (인덱스 배열)

        edges에 들어있는 i는 dist[i]와 dist[i+1] 사이가 크게 변한 경계
        """
        dist = np.asarray(dist, dtype=np.float32)
        if dist.size < 2:
            return np.array([], dtype=np.int32)

        diff = np.diff(dist)  # dist[i+1]-dist[i]
        edges = np.where(np.abs(diff) > jump_thresh)[0].astype(np.int32)
        return edges

    def pick_gap_from_edges(self, edges, total_len, min_width=5):
        """
        큰 변화 2번으로 "빈 공간" 구간 잡기
        edges: detect_jump_edges 결과
        min_width: gap으로 인정할 최소 인덱스 폭 (너무 좁으면 노이즈)
        return: (gap_start, gap_end) or None
          gap_start ~ gap_end 구간이 '빈 공간'이라고 가정
        """
        edges = np.asarray(edges, dtype=np.int32)
        if edges.size < 2:
            e = edges[0]
            gap_start = e + 1
            gap_end = total_len - 1
            if gap_end - gap_start >= min_width:
                return gap_start, gap_end
            else:
                return None

        e1 = edges[0]
        e2 = None
        for e in edges[1:]:
            if (e - e1) >= min_width:
                e2 = e
                break

        if e2 is None:
            return None

        gap_start = int(e1 + 1)
        gap_end = int(e2)  # inclusive로 쓸 수도 있지만 여기선 범위 느낌만
        if gap_end <= gap_start:
            return None
        return gap_start, gap_end

    def compute_gap_distance(self, angle, dist, gap_start, gap_end, method="center"):
        angle = np.asarray(angle, dtype=np.float32)
        dist = np.asarray(dist, dtype=np.float32)

        if angle.size == 0 or dist.size == 0:
            return None
        if gap_start < 0 or gap_end >= dist.size:
            return None

        # --- 수정 포인트: 인덱스 중앙이 아닌 '각도'의 산술 평균 ---
        start_angle = angle[gap_start]
        end_angle = angle[gap_end]
        gap_angle = float((start_angle + end_angle) / 2.0)

        if method == "center":
            # 거리값은 여전히 중앙 인덱스 근처를 참고하거나
            mid = (gap_start + gap_end) // 2
            gap_dist = float(dist[mid])
        elif method == "average":
            # 해당 구간의 거리 평균값 사용
            gap_dist = float(np.mean(dist[gap_start:gap_end + 1]))
        else:
            raise ValueError("method must be 'center' or 'average'")

        return gap_dist, gap_angle

    def angle_in_range(self, angle, center, tol):
        """
        angle: 0~360
        center: 기준 각도
        tol: ± 허용 범위
        """
        diff = abs((angle - center + 180) % 360 - 180)
        return diff <= tol

    def get_object_list(self, angle, dist, edges):
        objects = []
        # 확실하게 정수형으로 변환하여 인덱스 오류 방지
        idx_points = [0] + sorted(list(edges.astype(int))) + [len(dist)]

        for i in range(len(idx_points) - 1):
            start = int(idx_points[i])
            end = int(idx_points[i + 1])

            # 점의 개수가 너무 적으면 패스
            if (end - start) < 3:
                continue

            obj_dist = dist[start:end]
            obj_angle = angle[start:end]

            # 모든 값이 유효한지 확인
            average_d = np.average(obj_dist)

            # 음수 방지: median_d가 양수일 때만 추가
            if 10.0 < average_d < 8000.0:
                stats = {
                    "center_angle": float((obj_angle[0] + obj_angle[-1]) / 2.0),
                    "mean_dist": float(np.mean(obj_dist)),
                    "average_dist": float(average_d),
                    "width_points": int(end - start)
                }
                objects.append(stats)
        return objects
