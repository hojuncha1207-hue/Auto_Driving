"""
-------------------------------------------------------------------
 FILE NAME: Function_Library.py
 Copyright: Sungkyunkwan University, Automation Lab.
-------------------------------------------------------------------
 This file is included library class for below subject.
 1) Arduino
 2) LiDAR
 3) Camera
-------------------------------------------------------------------
 Authors: Jonghun Kim, YoungSoo Do, SungBhin Oh, HyeongKeun Hong

 Generated: 2022-11-10
 Revised: 2022-11-18
-------------------------------------------------------------------
 If you find some wrong code, plz contact me(Main Author: Jonghun Kim).
-------------------------------------------------------------------
 You should never modify this file during workshop exercise.
-------------------------------------------------------------------
"""

import sys
import cv2  # pip install opencv
import time
import serial  # pip install serial
import numpy as np  # pip install numpy
import matplotlib.pyplot as plt  # pip install matplotlib
from rplidar import RPLidar  # pip install rplidar-roboticia

np.set_printoptions(threshold=sys.maxsize, linewidth=150)

"""------------------Arduino Variable------------------"""
WAIT_TIME = 2
"""----------------------------------------------------"""

"""-------------------LIDAR Variable-------------------"""
SCAN_TYPE = "normal"
SAMPLE_RATE = 10
MAX_BUFFER_SIZE = 3000
MIN_DISTANCE = 0
"""----------------------------------------------------"""

"""--------------Computer Vision Variable--------------"""
NULL = 0
VARIANCE = 30
SATURATION = 150
FORWARD_THRESHOLD = 0.3
RED, GREEN, BLUE, YELLOW = (0, 1, 2, 3)
FORWARD, LEFT, RIGHT = (0, 1, 2)
COLOR = ("RED", "GREEN", "BLUE", "YELLOW")
DIRECTION = ("FORWARD", "LEFT", "RIGHT")
HUE_THRESHOLD = ([4, 176], [40, 80], [110, 130], [20, 40])
"""-----------------------------------------------------"""

"""
-------------------------------------------------------------------
 CLASS PURPOSE: Arduino Exercise Library
 Author: SungBhin Oh
 Revised: 2022-11-14
-------------------------------------------------------------------
"""


# noinspection PyMethodMayBeStatic
class libARDUINO(object):
    def __init__(self):
        self.port = None
        self.baudrate = None
        self.wait_time = WAIT_TIME  # second unit

    # Arduino Serial USB Port Setting
    def init(self, port, baudrate):
        ser = serial.Serial()
        ser.port, self.port = port, port
        ser.baudrate, self.baudrate = baudrate, baudrate
        ser.open()
        time.sleep(self.wait_time)
        return ser


"""
-------------------------------------------------------------------
 CLASS PURPOSE: LiDAR Sensor Exercise Library
 Author: YoungSoo Do
 Revised: 2022-11-18
-------------------------------------------------------------------
""""""
-------------------------------------------------------------------
 CLASS PURPOSE: LiDAR Sensor Exercise Library
 Author: YoungSoo Do
 Revised: 2022-11-18
-------------------------------------------------------------------
"""


class libLIDAR(object):
    def __init__(self, port):
        """
        라이다 센서 객체 생성 및 연결
        :param port: 라이다가 연결된 시리얼 포트 (예: 'COM3' 또는 '/dev/ttyUSB0')
        """
        self.rpm = 0
        self.lidar = RPLidar(port)
        self.scan = []

    def init(self):
        """
        라이다 장치 정보를 읽어와 출력 (연결 확인용)
        :return: None
        """
        info = self.lidar.get_info()
        print(info)

    def getState(self):
        """
        라이다 센서의 현재 상태(Health) 점검
        :return: None (상태 정보를 콘솔에 출력)
        """
        health = self.lidar.get_health()
        print(health)

    def scanning(self):
        """
        [핵심] 라이다 센서로부터 데이터를 계속해서 받아오는 제너레이터 함수
        - 360도 한 바퀴 스캔이 완료될 때마다 데이터를 반환(yield)합니다.

        :yield: [각도(Angle), 거리(Distance)] 형태의 Numpy 2차원 배열
                (Quality 값은 제외하고 각도와 거리만 반환함)
        """
        scan_list = []
        iterator = self.lidar.iter_measures(SCAN_TYPE, MAX_BUFFER_SIZE)

        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                # 데이터가 일정 개수(SAMPLE_RATE) 이상 모였을 때만 반환
                if len(scan_list) > SAMPLE_RATE:
                    np_data = np.array(list(scan_list))
                    # scan_list는 (quality, angle, distance) 순서임
                    # 여기서 [:, 1:] 슬라이싱을 통해 quality를 버리고 [angle, distance]만 남김
                    yield np_data[:, 1:]
                scan_list = []

            # 노이즈 필터링: 너무 가까운 거리(MIN_DISTANCE)는 무시
            if distance > MIN_DISTANCE:
                scan_list.append((quality, angle, distance))

    def stop(self):
        """
        라이다 작동 중지 및 연결 해제 (프로그램 종료 시 필수)
        """
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def setRPM(self, rpm):
        """
        라이다 모터 회전 속도 설정
        :param rpm: 설정할 RPM 값
        """
        self.lidar.motor_speed = rpm

    def getRPM(self):
        """
        현재 모터 회전 속도 조회
        :return: 현재 RPM 값
        """
        return self.lidar.motor_speed

    def getAngleRange(self, scan, minAngle, maxAngle):
        """
        특정 각도 범위 내의 데이터만 추출
        :param scan: scanning()에서 받은 [angle, distance] 데이터
        :param minAngle: 최소 각도 (0~360)
        :param maxAngle: 최대 각도 (0~360)
        :return: 범위 내 데이터만 남은 Numpy 배열
        """
        data = np.array(scan)
        # data[:, 0]은 Angle(각도) 열입니다.
        condition = np.where((data[:, 0] < maxAngle) & (data[:, 0] > minAngle))
        return data[condition]

    def getDistanceRange(self, scan, minDist, maxDist):
        """
        특정 거리 범위 내의 데이터만 추출
        :param scan: scanning()에서 받은 [angle, distance] 데이터
        :param minDist: 최소 거리 (mm 단위)
        :param maxDist: 최대 거리 (mm 단위)
        :return: 범위 내 데이터만 남은 Numpy 배열
        """
        data = np.array(scan)
        # data[:, 1]은 Distance(거리) 열입니다.
        condition = np.where((data[:, 1] < maxDist) & (data[:, 1] > minDist))
        return data[condition]

    def getAngleDistanceRange(self, scan, minAngle, maxAngle, minDist, maxDist):
        """
        [필터링 끝판왕] 특정 각도 AND 특정 거리 범위 내의 데이터만 추출
        예: "전방 30도(각도) 안에 있는 1미터 이내(거리) 장애물만 보여줘"

        :param scan: 원본 데이터
        :param minAngle: 최소 각도
        :param maxAngle: 최대 각도
        :param minDist: 최소 거리
        :param maxDist: 최대 거리
        :return: 조건을 모두 만족하는 데이터 Numpy 배열
        """
        data = np.array(scan)
        condition = np.where(
            (data[:, 0] < maxAngle) & (data[:, 0] > minAngle) & (data[:, 1] < maxDist) & (data[:, 1] > minDist))
        return data[condition]


"""
-------------------------------------------------------------------
 CLASS PURPOSE: Camera Sensor Exercise Library
 Author: Jonghun Kim
 Revised: 2022-11-12
-------------------------------------------------------------------
"""


# noinspection PyMethodMayBeStatic
class libCAMERA(object):
    def __init__(self):
        """
        카메라 라이브러리 초기화
        """
        self.capnum = 0
        self.row, self.col, self.dim = (0, 0, 0)

    def loop_break(self):
        """
        'q' 키를 누르면 루프를 종료하는 신호를 보냄
        :return: True(종료) / False(계속)
        """
        if cv2.waitKey(10) & 0xFF == ord('q'):
            print("Camera Reading is ended.")
            return True
        else:
            return False

    def file_read(self, img_path):
        """
        이미지 파일을 읽어옴
        :param img_path: 파일 경로 (예: "./image.jpg")
        :return: 이미지 배열 (numpy array)
        """
        return np.array(cv2.imread(img_path))

    def rgb_conversion(self, img):
        """
        BGR 이미지를 RGB로 변환
        :param img: 원본 BGR 이미지
        :return: 변환된 RGB 이미지
        """
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)

    def hsv_conversion(self, img):
        """
        BGR 이미지를 HSV로 변환
        :param img: 원본 BGR 이미지
        :return: 변환된 HSV 이미지
        """
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)

    def gray_conversion(self, img):
        """
        컬러 이미지를 흑백(Grayscale)으로 변환
        :param img: 원본 컬러 이미지
        :return: 변환된 흑백 이미지
        """

        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)

    def color_extract(self, img, idx):
        """
        특정 색상 채널(R, G, B)만 남기고 나머지는 제거
        :param img: 원본 이미지
        :param idx: 남길 채널 인덱스 (RED=0, GREEN=1, BLUE=2)
        :return: 특정 채널만 남은 이미지
        """
        result = img.copy()

        for i in range(RED + GREEN + BLUE):
            if i != idx:
                result[:, :, i] = np.zeros([self.row, self.col])

        return result

    def extract_rgb(self, img, print_enable=False):
        """
        이미지를 R, G, B 채널별로 분리하여 반환
        :param img: 원본 이미지
        :param print_enable: 분리된 채널을 그래프로 출력할지 여부
        :return: (Red채널, Green채널, Blue채널) 튜플
        """
        self.row, self.col, self.dim = img.shape

        img = self.rgb_conversion(img)

        # Image Color Separating
        img_red = self.color_extract(img, RED)
        img_green = self.color_extract(img, GREEN)
        img_blue = self.color_extract(img, BLUE)

        if print_enable:
            plt.figure(figsize=(12, 4))
            imgset = [img_red, img_green, img_blue]
            imglabel = ["RED", "GREEN", "BLUE"]

            for idx in range(RED + GREEN + BLUE):
                plt.subplot(1, 3, idx + 1)
                plt.xlabel(imglabel[idx])
                plt.imshow(imgset[idx])
            plt.show()

        return img_red[:, :, RED], img_green[:, :, GREEN], img_blue[:, :, BLUE]

    def initial_setting(self, cam0port=0, cam1port=1, capnum=1):
        """
        카메라 장치 연결 및 초기화
        :param cam0port: 1번 카메라 포트 번호 (기본 0)
        :param cam1port: 2번 카메라 포트 번호 (기본 1)
        :param capnum: 사용할 카메라 개수 (1 or 2)
        :return: (cam0객체, cam1객체)
        """
        # OpenCV Initial Setting
        print("OpenCV Version:", cv2.__version__)
        channel0 = None
        channel1 = None
        self.capnum = capnum

        if capnum == 1:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")
        elif capnum == 2:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")

            channel1 = cv2.VideoCapture(cv2.CAP_DSHOW + cam1port)
            if channel1.isOpened():
                print("Camera Channel1 is enabled!")

        return channel0, channel1

    def camera_read(self, cap1, cap2=None):
        """
        연결된 카메라로부터 프레임을 읽어옴
        :param cap1: 1번 카메라 객체
        :param cap2: 2번 카메라 객체 (선택)
        :return: [성공여부1, 프레임1, 성공여부2, 프레임2...] 리스트
        """
        result, capset = [], [cap1, cap2]

        for idx in range(0, self.capnum):
            ret, frame = capset[idx].read()
            result.extend([ret, frame])

        return result

    def image_show(self, frame0, frame1=None):
        """
        이미지를 화면에 띄움
        :param frame0: 첫 번째 이미지
        :param frame1: 두 번째 이미지 (선택)
        """
        if frame1 is None:
            cv2.imshow('frame0', frame0)
        else:
            cv2.imshow('frame0', frame0)
            cv2.imshow('frame1', frame1)

    def color_filtering(self, img, roi=None, print_enable=False):
        """
        HSV 색상 공간을 이용해 특정 색상만 필터링
        :param img: 원본 이미지
        :param roi: 필터링할 목표 색상 (RED, YELLOW, GREEN 등)
        :param print_enable: 결과 이미지를 출력할지 여부
        :return: 필터링된 이미지
        """
        self.row, self.col, self.dim = img.shape

        hsv_img = self.hsv_conversion(img)
        h, s, v = cv2.split(hsv_img)

        s_cond = s > SATURATION
        if roi is RED:
            h_cond = (h < HUE_THRESHOLD[roi][0]) | (h > HUE_THRESHOLD[roi][1])
        else:
            h_cond = (h > HUE_THRESHOLD[roi][0]) & (h < HUE_THRESHOLD[roi][1])

        v[~h_cond], v[~s_cond] = 0, 0
        hsv_image = cv2.merge([h, s, v])
        result = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

        if print_enable:
            self.image_show(result)

        return result

    def gaussian_blurring(self, img, kernel_size=(None, None)):
        """
        가우시안 블러 처리 (노이즈 제거)
        :param img: 원본 이미지
        :param kernel_size: 커널 크기 (예: (5, 5))
        :return: 블러 처리된 이미지
        """
        return cv2.GaussianBlur(img.copy(), kernel_size, 0)

    def canny_edge(self, img, lth, hth):
        """
        Canny 알고리즘으로 에지(외곽선) 검출
        :param img: 원본 이미지
        :param lth: 하위 임계값 (Low Threshold)
        :param hth: 상위 임계값 (High Threshold)
        :return: 에지 이미지
        """
        return cv2.Canny(img.copy(), lth, hth)

    def histogram_equalization(self, gray):
        """
        히스토그램 평활화 (명암비 개선)
        :param gray: 흑백 이미지
        :return: 평활화된 이미지
        """
        return cv2.equalizeHist(gray)

    def hough_transform(self, img, rho=None, theta=None, threshold=None, mll=None, mlg=None, mode="lineP"):
        """
        허프 변환을 이용한 도형(직선, 원) 검출
        :param img: 입력 이미지 (주로 에지 이미지)
        :param rho: 거리 해상도 (픽셀 단위)
        :param theta: 각도 해상도 (라디안 단위)
        :param threshold: 임계값 (직선/원 인정 기준 투표수)
        :param mll: 최소 선 길이 (minLineLength)
        :param mlg: 최대 선 간격 (maxLineGap)
        :param mode: "line"(일반허프), "lineP"(확률적허프), "circle"(원)
        :return: 검출된 도형 정보 (직선 좌표들 또는 원 정보들)
        """
        if mode == "line":
            return cv2.HoughLines(img.copy(), rho, theta, threshold)
        elif mode == "lineP":
            return cv2.HoughLinesP(img.copy(), rho, theta, threshold, lines=np.array([]),
                                   minLineLength=mll, maxLineGap=mlg)
        elif mode == "circle":
            return cv2.HoughCircles(img.copy(), cv2.HOUGH_GRADIENT, dp=1, minDist=80,
                                    param1=200, param2=10, minRadius=40, maxRadius=100)

    def morphology(self, img, kernel_size=(None, None), mode="opening"):
        """
        형태학적 변환 (Morphology) 수행
        :param img: 입력 이미지
        :param kernel_size: 커널 크기
        :param mode: "opening"(잡음제거), "closing"(구멍메우기), "gradient"(외곽선)
        :return: 변환된 이미지
        """
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)

        if mode == "opening":
            dst = cv2.erode(img.copy(), kernel)
            return cv2.dilate(dst, kernel)
        elif mode == "closing":
            dst = cv2.dilate(img.copy(), kernel)
            return cv2.erode(dst, kernel)
        elif mode == "gradient":
            return cv2.morphologyEx(img.copy(), cv2.MORPH_GRADIENT, kernel)

    def point_analyze(self, gray, line, point_gap, len_threshold):
        """
        검출된 선 주변의 픽셀값 차이를 분석하여 유효성 검증
        :param gray: 흑백 이미지
        :param line: 검사할 직선 좌표 [x1, y1, x2, y2]
        :param point_gap: 선 주변 픽셀을 볼 간격
        :param len_threshold: 인정할 최소 명암 차이
        :return: True(유효함) / False(무효함)
        """
        disparity = [0, 0]

        for idx in range(2):
            yplus = line[idx + 1] + point_gap if line[idx + 1] + point_gap < self.row else self.row - 1
            yminus = line[idx + 1] - point_gap if line[idx + 1] - point_gap >= 0 else 0

            if yplus < 0 or yminus >= self.row:
                break
            elif yplus >= self.row or yminus < 0:
                break

            disparity[idx] = np.abs(gray[yplus][line[idx]] - gray[yminus][line[idx]])

        if np.average(disparity) > len_threshold:
            return True
        else:
            return False

    def object_detection(self, img, sample=0, mode="circle", print_enable=False):
        """
        신호등 색상 및 원형 검출
        :param img: 입력 이미지
        :param sample: 원 내부 색상 검증을 위한 샘플링 범위
        :param mode: 도형 검출 모드 ("circle")
        :param print_enable: 결과 출력 여부
        :return: 감지된 신호등 색상 문자열 (예: "RED") 또는 None
        """
        result = None
        replica = img.copy()

        for color in (RED, YELLOW, GREEN):
            extract = self.color_filtering(img, roi=color, print_enable=True)
            gray = self.gray_conversion(extract)
            circles = self.hough_transform(gray, mode=mode)
            if circles is not None:
                for circle in circles[0]:
                    center, count = (int(circle[0]), int(circle[1])), 0

                    hsv_img = self.hsv_conversion(img)
                    h, s, v = cv2.split(hsv_img)

                    # Searching the surrounding pixels
                    for res in range(sample):
                        x, y = int(center[1] - sample / 2), int(center[0] - sample / 2)
                        s_cond = s[x][y] > SATURATION
                        if color is RED:
                            h_cond = (h[x][y] < HUE_THRESHOLD[color][0]) | (h[x][y] > HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count
                        else:
                            h_cond = (h[x][y] > HUE_THRESHOLD[color][0]) & (h[x][y] < HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count

                    if count > sample / 2:
                        result = COLOR[color]
                        cv2.circle(replica, center, int(circle[2]), (0, 0, 255), 2)

        if print_enable:
            if result is not None:
                print("Traffic Light: ", result)
            self.image_show(replica)

        return result

    def edge_detection(self, img, width=0, height=0, gap=0, threshold=0, print_enable=False):
        """
        차선 인식 및 주행 방향 판단 (Canny Edge + Hough Line)
        :param img: 입력 이미지
        :param width: 기울기 필터링 기준 (가로폭 제한)
        :param height: 길이 필터링 기준 (세로길이 제한)
        :param gap: point_analyze에 사용될 픽셀 간격
        :param threshold: point_analyze에 사용될 임계값
        :param print_enable: 결과 출력 여부
        :return: 주행 방향 (FORWARD=0, LEFT=1, RIGHT=2) 또는 None
        """
        prediction = None
        replica = img.copy()
        self.row, self.col, self.dim = img.shape

        gray_scale = self.gray_conversion(img)
        hist = self.histogram_equalization(gray_scale)
        dst = self.morphology(hist, (2, 2), mode="opening")

        blurring = self.gaussian_blurring(dst, (5, 5))
        canny = self.canny_edge(blurring, 100, 200)

        lines = self.hough_transform(canny, 1, np.pi / 180, 50, 10, 20, mode="lineP")

        if lines is not None:
            new_lines, real_lines = [], []
            for line in lines:
                xa, ya, xb, yb = line[0]

                # x range : 0 ~ self.col / y range : 0 ~ self.row
                if np.abs(yb - ya) > height and np.abs(xb - xa) < width:
                    if self.point_analyze(blurring, line[0], gap, threshold):
                        for idx in range(len(new_lines)):
                            if np.abs(new_lines[:][idx][1] - ya) < VARIANCE:
                                if np.abs(new_lines[:][idx][3] - yb) < VARIANCE:

                                    grad = (xb - xa) / -(yb - ya)  # the third quadrant

                                    if np.abs(grad) < FORWARD_THRESHOLD:
                                        prediction = FORWARD
                                    elif grad > 0:
                                        prediction = RIGHT
                                    elif grad < 0:
                                        prediction = LEFT

                                    # real_lines.append([xa, ya, xb, yb])
                                    cv2.line(replica, (xa, ya), (xb, yb), color=[0, 0, 255], thickness=2)
                        new_lines.append([xa, ya, xb, yb])
            if print_enable:
                if prediction is not None:
                    print("Vehicle Direction: ", DIRECTION[prediction])
                self.image_show(replica)

        return prediction

