import parking_lib as pl
import serial
import time
import math

arduino = serial.Serial(port="COM4", baudrate=9600, timeout=1)


def send_to_arduino(area, dist):
    # 데이터를 "구역,거리\n" 형식의 한 줄로 만듭니다.
    data = f"{area},{dist:.2f}\n"
    arduino.write(data.encode())  # 바이트로 변환하여 전송
    print(f"[SERIAL] Sent to Arduino: {data.strip()}")

def main():
    # LiDAR 연결을 시작합니다.
    lidar_lib = pl.libLIDAR(port="COM3")

    if not getattr(lidar_lib, "is_connected", True):
        print("LIDAR connection failed.")
        return

    # ---- [통합 튜닝 포인트] 두 번째 코드의 민감도 채택 ----
    ANG_MIN, ANG_MAX = 0.0, 70.0
    MIN_DIST = 150.0
    SMOOTH_K = 3
    JUMP_THRESH = 150.0
    MIN_POINTS = 2
    TARGET_CENTER = 35.0
    TARGET_TOL = 25.0

    # 아래 범위 내로 탐지
    # 두번째 주차 공간 범위
    Parking_area2_dist_MIN = 2600
    Parking_area2_dist_MAX = 3000

    # 세번째 주차 공간 범위
    Parking_area3_dist_MIN = 3600
    Parking_area3_dist_MAX = 4000

    try:
        print("Starting measurement... Press Ctrl+C to stop.")
        # 루프를 하나로 합쳐 실시간 데이터 처리를 극대화합니다.
        for scan_data in lidar_lib.scanning():
            angle, dist = lidar_lib.extract_angle_distance(scan_data)

            # 1) 섹터 선택 및 정렬
            a, d = lidar_lib.select_and_sort_sector(angle, dist, ANG_MIN, ANG_MAX, min_dist=MIN_DIST)
            if d.size < MIN_POINTS:
                continue

            # 2) 전처리 (Smoothing) 및 경계(Edge) 탐지
            d_s = lidar_lib.median_smooth_1d(d, k=SMOOTH_K)
            edges = lidar_lib.detect_jump_edges(d_s, jump_thresh=JUMP_THRESH)

            # 3) 물체 개별 추출 (get_object_list 활용)
            all_clusters = lidar_lib.get_object_list(a, d_s, edges)
            # 6m 이내의 유효한 물체만 필터링
            detected_objects = [obj for obj in all_clusters if obj['average_dist'] < 6000]

            # 4) 물체가 2개 미만이면 조건을 만족할 수 없으므로 재측정
            if len(detected_objects) < 2:
                print("물체 부족으로 재측정 중...")
                continue

            # 5) 물체 1과 물체 2 사이의 물리적 관계 계산 (승하님의 핵심 로직)
            obj1 = detected_objects[0]
            obj2 = detected_objects[1]

            # 두 물체의 정중앙 각도 평균
            real_gap_angle = (obj1['center_angle'] + obj2['center_angle']) / 2.0
            # 두 물체의 평균 거리 (빗변 r)
            real_gap_dist = (obj1['average_dist'] + obj2['average_dist']) / 2.0

            # [핵심] 직선 거리(수직 거리) 계산: d = r * cos(theta)
            # 파이썬 math.cos는 라디안을 사용하므로 radians() 변환 필수
            real_gap_straight_dist = round(round(math.cos(math.radians(real_gap_angle)), 4) * real_gap_dist,2)

            # 6) 특정 주차 공간 거리 범위 필터링 (2700~2900 또는 3800~4000)
            is_in_range = (Parking_area2_dist_MIN <= real_gap_straight_dist <= Parking_area2_dist_MAX) \
                          or (Parking_area3_dist_MIN <= real_gap_straight_dist <= Parking_area3_dist_MAX)

            if not is_in_range:
                # 조건에 맞지 않으면 측정값만 간략히 출력하고 다시 루프
                print(f"[SEARCHING] 직선거리: {real_gap_straight_dist:.1f}mm - 범위 밖")
                continue

            # 7) [성공] 조건에 부합하는 경우 상세 데이터 출력
            print("-" * 50)
            print(f"물체 1: 각도={obj1['center_angle']:.1f}°, 거리={obj1['average_dist']:.1f}mm")
            print(f"물체 2: 각도={obj2['center_angle']:.1f}°, 거리={obj2['average_dist']:.1f}mm")
            print(f"성공! 두 개의 물체 사이 공간 확보")


            parking_area = 0
            # 주차 구역 판별

            if (Parking_area2_dist_MIN <= real_gap_straight_dist <= Parking_area2_dist_MAX) or (Parking_area3_dist_MIN <= real_gap_straight_dist <= Parking_area3_dist_MAX) :
                print("front")
            elif (Parking_area2_dist_MIN + 300 <= real_gap_straight_dist <= Parking_area2_dist_MAX + 300) or (Parking_area3_dist_MIN + 300 <= real_gap_straight_dist <= Parking_area3_dist_MAX + 300):
                print("back")
            else:
                continue

            if Parking_area2_dist_MIN <= real_gap_straight_dist <= Parking_area2_dist_MAX:
                print(">>> [RESULT] 두번째 주차공간 확정 (Area 1)")
                parking_area = 2

            elif Parking_area3_dist_MIN <= real_gap_straight_dist <= Parking_area3_dist_MAX:
                print(">>> [RESULT] 세번째 주차공간 확정 (Area 2)")
                parking_area = 3

            print(f"[최종 빈공간] 직선거리={real_gap_straight_dist:.1f}mm @ 중앙각={real_gap_angle:.1f}°")

            if parking_area > 0:
                # 함수 한 번 호출로 두 데이터를 콤마로 묶어 보냅니다.
                send_to_arduino(parking_area, real_gap_straight_dist)
            else:
                # 감지 안 될 때는 0,0으로 보냅니다.
                send_to_arduino(0, 0.0)

            # 최종 각도 타겟 검사 및 종료
            if lidar_lib.angle_in_range(real_gap_angle, TARGET_CENTER, TARGET_TOL):
                print(f"🔴 [TARGET FOUND] 거리와 각도 모두 일치! 시스템을 종료합니다.")
                break
            else:
                print(f"[RETRY] 거리는 맞지만 각도가 타겟 범위 밖입니다: {real_gap_angle:.1f}°")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStop sending. Stopping...")
    finally:
        if hasattr(lidar_lib, "stop"):
            lidar_lib.stop()


if __name__ == "__main__":
    main()
