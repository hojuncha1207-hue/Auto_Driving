import time
from rplidar import RPLidar


class LidarObjectDetector:
    def __init__(self, port='COM5', baudrate=9600):
        self.is_connected = False
        self.lidar = None
        self.scan_generator = None

        try:
            # timeout�� ª�� �����Ͽ� ������ ���� �� ���� ������ ���� ����ϴ� ���� ����
            self.lidar = RPLidar(port, baudrate=baudrate, timeout=0.5)
            self.lidar.connect()
            self.lidar.start_motor()
            # max_buf_meas�� ���� ũ���Դϴ�.
            self.scan_generator = self.lidar.iter_scans(max_buf_meas=500)
            self.is_connected = True
            print(f"LiDAR connected on {port}")
        except Exception as e:
            print(f"LiDAR Connection Error: {e}")

    def get_obstacle_status(self, min_angle=-30, max_angle=30, max_dist=1000, min_points=10):
        """
        ��ֹ� ���� ����, ������ ����Ʈ ����, ��� �Ÿ��� ��ȯ�մϴ�.
        :param min_angle: ���� ���� ���� (������ 0��, ������ ����)
        :param max_angle: ���� ���� ����
        :param max_dist: ���� �ִ� �Ÿ� (mm)
        :param min_points: ��ֹ��� �ν��ϱ� ���� �ּ� ����Ʈ ����
        """
        if not self.is_connected or self.scan_generator is None:
            return False, 0, 0.0

        try:
            # �ֽ� ��ĵ ������ �� ������ �������� (Blocking �߻� ����)
            scan = next(self.scan_generator)

            obstacle_count = 0
            total_dist = 0.0

            for (_, angle, distance) in scan:
                if distance <= 0: continue  # ���� ������ ����

                # ���� ���� (0~360 -> -180~180)
                adj_angle = angle if angle <= 180 else angle - 360

                # ������ ���� ���� ���� �ִ��� Ȯ��
                if min_angle <= adj_angle <= max_angle:
                    # �ʹ� ����� ������(15cm �̸�)�� �����ϰ� max_dist �̳��� üũ
                    if 300 <= distance <= max_dist:
                        obstacle_count += 1
                        total_dist += distance

            # ������ ����Ʈ�� ������ �̻��� ���� ��ֹ��� �Ǵ�
            if obstacle_count >= min_points:
                avg_dist = total_dist / obstacle_count
                return True, obstacle_count, avg_dist
            else:
                return False, obstacle_count, 0.0

        except StopIteration:
            # ���̴ٰ� ���� �� ������ �� �� ���Ƽ� �����Ͱ� ���� ���
            return False, 0, 0.0
        except Exception as e:
            # ��� ���� �� ��Ÿ ���� ó��
            # print(f"LiDAR Read Error: {e}")
            return False, 0, 0.0

    def stop(self):
        """���̴� �����ϰ� ����"""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print(" LiDAR safely disconnected.")
            except:
                pass


