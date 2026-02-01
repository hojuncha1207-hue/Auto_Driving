import time
from rplidar import RPLidar


class LidarObjectDetector:
    def __init__(self, port='COM5', baudrate=9600):
        self.is_connected = False
        self.lidar = None
        self.scan_generator = None

        try:
            # timeout을 짧게 설정하여 데이터 지연 시 메인 루프가 오래 대기하는 것을 방지
            self.lidar = RPLidar(port, baudrate=baudrate, timeout=0.5)
            self.lidar.connect()
            self.lidar.start_motor()
            # max_buf_meas는 버퍼 크기입니다.
            self.scan_generator = self.lidar.iter_scans(max_buf_meas=500)
            self.is_connected = True
            print(f"LiDAR connected on {port}")
        except Exception as e:
            print(f"LiDAR Connection Error: {e}")

    def get_obstacle_status(self, min_angle=-30, max_angle=30, max_dist=1000, min_points=10):
        """
        장애물 감지 여부, 감지된 포인트 개수, 평균 거리를 반환합니다.
        :param min_angle: 감지 시작 각도 (정면이 0도, 왼쪽이 음수)
        :param max_angle: 감지 종료 각도
        :param max_dist: 감지 최대 거리 (mm)
        :param min_points: 장애물로 인식하기 위한 최소 포인트 개수
        """
        if not self.is_connected or self.scan_generator is None:
            return False, 0, 0.0

        try:
            # 최신 스캔 데이터 한 바퀴분 가져오기 (Blocking 발생 가능)
            scan = next(self.scan_generator)

            obstacle_count = 0
            total_dist = 0.0

            for (_, angle, distance) in scan:
                if distance <= 0: continue  # 측정 오류값 무시

                # 각도 보정 (0~360 -> -180~180)
                adj_angle = angle if angle <= 180 else angle - 360

                # 설정된 각도 범위 내에 있는지 확인
                if min_angle <= adj_angle <= max_angle:
                    # 너무 가까운 노이즈(15cm 미만)는 무시하고 max_dist 이내만 체크
                    if 300 <= distance <= max_dist:
                        obstacle_count += 1
                        total_dist += distance

            # 감지된 포인트가 설정값 이상일 때만 장애물로 판단
            if obstacle_count >= min_points:
                avg_dist = total_dist / obstacle_count
                return True, obstacle_count, avg_dist
            else:
                return False, obstacle_count, 0.0

        except StopIteration:
            # 라이다가 아직 한 바퀴를 다 못 돌아서 데이터가 없는 경우
            return False, 0, 0.0
        except Exception as e:
            # 통신 에러 등 기타 예외 처리
            # print(f"LiDAR Read Error: {e}")
            return False, 0, 0.0

    def stop(self):
        """라이다 안전하게 종료"""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print(" LiDAR safely disconnected.")
            except:
                pass


