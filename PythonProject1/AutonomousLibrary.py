import cv2
import numpy as np
import math


left_roi_pts = np.float32([
  [0, 480], [300, 480], [240, 230], [50, 230]
])
right_roi_pts = np.float32([
  [340, 480], [640, 480], [590, 230], [400, 230]
])


GAP_TOP, HEIGHT_TOP, X_OFFSET = 160, 150,100
LANE_WIDTH_PIXELS = 355
class LaneDetector:
  def __init__(self, width=640, height=480):
      self.width = width
      self.height = height
      # [추가] BEV 상에서의 차선 폭 (픽셀 단위, 환경에 맞춰 튜닝 필요)
      self.lane_width = LANE_WIDTH_PIXELS


  def mask_green_floor(self, img):
      """초록색 바닥 영역을 검은색으로 마스킹"""


      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      lower_green = np.array([35, 35, 35])  # (H: 35->30, S: 50->20, V: 50->20)
      upper_green = np.array([85, 255, 255])  # (H: 85->100)


      # 3. 마스크 생성 (초록색 영역만 흰색인 이미지)
      green_mask = cv2.inRange(hsv, lower_green, upper_green)
      # green_mask 생성 직후에 추가
      kernel = np.ones((3, 3), np.uint8)
      # 마스크 내부의 작은 구멍들을 메우고 경계를 부드럽게 함
      green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
      green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_DILATE, kernel)  # 살짝 더 넓게 지우기
      # 4. 원본 이미지에서 초록색 영역을 검은색으로 칠하기
      masked_img = img.copy()
      # 마스크가 흰색(255)인 위치의 픽셀을 모두 검은색[0,0,0]으로 변경
      masked_img[green_mask != 0] = [0, 0, 0]


      return masked_img ,green_mask# 디버깅을 위해 마스크도 반환


  def erase_right_of_green(self, img, green_mask):
      """초록색 마스크가 시작되는 지점부터 오른쪽을 모두 검은색으로 제거"""
      h, w = green_mask.shape
      result_img = img.copy()


      # 각 행(y)에 대해 초록색(255)이 처음 나타나는 위치(x)를 찾음
      # np.argmax는 해당 행에서 가장 큰 값(255)이 처음 나타나는 인덱스를 반환합니다.
      # 초록색이 없는 행은 0을 반환하므로 체크가 필요합니다.


      found_green = np.any(green_mask > 0, axis=1)  # 초록색이 존재하는 행들
      first_green_indices = np.argmax(green_mask > 0, axis=1)  # 각 행별 첫 초록색 x좌표


      for y in range(h):
          if found_green[y]:
              # 해당 줄에서 초록색이 시작되는 x좌표부터 끝까지 검정색(0,0,0) 처리
              x_start = first_green_indices[y]+3#3픽셀 옆에서 부터
              result_img[y, x_start:] = [0, 0, 0]


      return result_img


  def get_bev(self, frame, gap_top, height_top, x_offset):
      """사다리꼴 ROI를 직사각형 평면으로 펼침 (Bird's Eye View)"""
      pt_x_center = self.width / 2
      src_pts = np.float32([
          [0, self.height], [self.width, self.height],
          [pt_x_center + gap_top, height_top], [pt_x_center - gap_top, height_top]
      ])
      dst_pts = np.float32([
          [x_offset, self.height], [self.width - x_offset, self.height],
          [self.width - x_offset, 0], [x_offset, 0]
      ])
      M = cv2.getPerspectiveTransform(src_pts, dst_pts)
      warped = cv2.warpPerspective(frame, M, (self.width, self.height))
      return warped, src_pts


  def fill_bev_dead_zones(self, bev_img, y_start=400):
      """
      BEV 이미지의 하단 빈 삼각형 영역을 윗줄 픽셀로 수직 확장하여 채움
      y_start: 확장을 시작할 높이 (보통 삼각형이 시작되는 지점)
      """
      h, w = bev_img.shape[:2]
      filled_img = bev_img.copy()


      # y_start 지점부터 맨 아래까지 한 줄씩 검사
      for y in range(y_start, h):
          # 현재 줄(y)에서 0인 부분(검은색)을 찾아 윗 줄(y-1)의 값으로 덮어씌움
          # binary 이미지일 경우와 컬러 이미지일 경우 모두 대응 가능하도록 처리
          if len(filled_img.shape) == 3:  # 컬러 이미지 (3채널)
              mask = np.all(filled_img[y] == 0, axis=-1)
              filled_img[y][mask] = filled_img[y - 1][mask]
          else:  # 이진 이미지 (1채널)
              mask = (filled_img[y] == 0)
              filled_img[y][mask] = filled_img[y - 1][mask]


      return filled_img


  def detect_line_type(self, valid_windows_list, min_ratio=0.6):
      """
      각 윈도우의 유효성 정보를 바탕으로 차선 타입 판별
      valid_windows_list: [True, True, False, True, ...] 형태의 리스트
      min_ratio: 실선으로 판단할 최소 비율 (예: 0.6 = 60% 이상)
      """
      if not valid_windows_list:
          return "Lost"


      # 유효한(True) 윈도우의 개수를 셉니다.
      num_valid = sum(valid_windows_list)
      total_windows = len(valid_windows_list)


      # 비율 계산
      fill_ratio = num_valid / total_windows if total_windows > 0 else 0


      # 비율에 따라 타입 결정 (튜닝 필요)
      if fill_ratio > min_ratio:
          return "Solid"  # 실선
      elif fill_ratio > 0.2: # 최소한의 비율은 넘어야 점선으로 인정
          return "Dashed" # 점선
      else:
          return "Lost"   # 놓침


  def extend_pixels_side_only(self, binary_img, y_start=380, center_gap=100):
      """
      중앙 부분을 제외하고 좌/우측 하단 빈 공간만 수직 확장함.
      center_gap: 중앙에서 무시할 영역의 전체 가로 폭 (픽셀 단위)
      """
      h, w = binary_img.shape[:2]
      filled_img = binary_img.copy()


      # 중앙 제외를 위한 경계선 계산
      left_boundary = (w // 2) - (center_gap // 2)
      right_boundary = (w // 2) + (center_gap // 2)


      for y in range(y_start, h):
          # 1. 현재 줄에서 값이 0(검정)인 마스크 생성
          zero_mask = (filled_img[y] == 0)


          # 2. 중앙 영역(left_boundary ~ right_boundary)은 마스크에서 제외 (False 처리)
          # 즉, 중앙 부분은 0이어도 윗줄을 복사해오지 않음
          zero_mask[left_boundary:right_boundary] = False


          # 3. 마스킹된 영역(좌/우 사이드)만 윗줄 값 복사
          filled_img[y][zero_mask] = filled_img[y-1][zero_mask]


      return filled_img


  def get_binary_hls(self, img, gap_top=GAP_TOP, height_top=HEIGHT_TOP, x_offset=X_OFFSET, threshold=160):
      """HLS 필터링과 Canny Edge를 결합하여 차선 추출"""


      # 1. 원본 이미지 BEV 변환 (Canny용)
      orbev_img, _ = self.get_bev(img, gap_top, height_top, x_offset)


      # 2. HLS 필터링 (원본 이미지 기준)
      hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
      white_lane_binary = np.zeros_like(hls[:, :, 1])
      # 밝기(L)는 높고 채도(S)는 낮은 영역 추출 (흰색 차선)
      white_lane_binary[(hls[:, :, 1] > 150)] = 255


      # 3. 색상 필터링된 이미지 BEV 변환
      bev_binary, _ = self.get_bev(white_lane_binary, gap_top, height_top, x_offset)


      # 4. Canny Edge 추출 (이미 변환된 BEV 컬러 이미지 기반)
      gray_bev = cv2.cvtColor(orbev_img, cv2.COLOR_BGR2GRAY)
      blurred = cv2.GaussianBlur(gray_bev, (5, 5), 0)
      canny_edge = cv2.Canny(blurred, 50, 150)


      # 5. 색상 정보(bev_binary) + 형태 정보(canny_edge) 결합
      combined = cv2.bitwise_or(bev_binary, canny_edge)


      return combined


  def block_filter(self, binary_img, nwindows=40, min_w=5, max_w=9):
      """직사각형 블록 단위로 쪼개어 차선 두께(너비)가 비정상적인 객체 제거"""
      h_img, w_img = binary_img.shape
      window_height = h_img // nwindows
      filtered_img = np.zeros_like(binary_img)


      for i in range(nwindows):
          y_low, y_high = i * window_height, (i + 1) * window_height
          strip = binary_img[y_low:y_high, :]
          nlabels, labels, stats, _ = cv2.connectedComponentsWithStats(strip)
          for j in range(1, nlabels):
              w = stats[j, cv2.CC_STAT_WIDTH]
              if min_w <= w <= max_w:
                  filtered_img[y_low:y_high, :][labels == j] = 255
      return filtered_img


  def filter_by_shape(self, binary_img):
      nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_img)
      filtered_img = np.zeros_like(binary_img)


      for i in range(1, nlabels):
          w = stats[i, cv2.CC_STAT_WIDTH]
          h = stats[i, cv2.CC_STAT_HEIGHT]
          area = stats[i, cv2.CC_STAT_AREA]


          # 1. 가로선 제거: 세로가 가로보다 길어야 함 (비율 0.7 이상)
          aspect_ratio = h / w
          if aspect_ratio < 0.2: continue  # 민김도 조절 곡선??


          # 2. 면적 기준 완화: 점선이나 멀리 있는 차선은 300보다 작을 수 있음
          if area < 50:
              continue


          # 3. 최대 두께 제한 해제 또는 확대: 커브에서 뭉친 차선 대응
          if w < 5:
              continue


          filtered_img[labels == i] = 255
      return filtered_img


  def apply_dual_roi_mask(self, binary_img, left_pts, right_pts):
      """이진 이미지에 좌우 개별 ROI를 적용하여 관심 영역만 남김"""
      mask = np.zeros_like(binary_img)
      # 좌측 ROI 영역 생성 (흰색 255)
      cv2.fillPoly(mask, [np.int32(left_pts)], 255)
      # 우측 ROI 영역 생성 (흰색 255)
      cv2.fillPoly(mask, [np.int32(right_pts)], 255)


      # 원본 이진 이미지와 마스크 합성
      return cv2.bitwise_and(binary_img, mask)


  def draw_dual_roi(self, img, left_pts, right_pts):
      """원본 영상에 좌우 ROI 테두리를 시각화"""
      draw_img = img.copy() # 원본 보존을 위해 복사본 사용
      # 왼쪽은 빨간색(Red), 오른쪽은 파란색(Blue) 테두리 그리기
      cv2.polylines(draw_img, [np.int32(left_pts)], True, (0, 0, 255), 3)
      cv2.polylines(draw_img, [np.int32(right_pts)], True, (255, 0, 0), 3)
      return draw_img


  def sliding_window(self, binary_img, left_pts, right_pts, nwindows=12, margin=60, minpix=50):
      """이전 윈도우 위치를 기준으로 바로 윗 영역을 탐색하는 수직 추적 방식"""
      out_img = np.dstack((binary_img, binary_img, binary_img))
      window_height = int(self.height // nwindows)


      # [1] 시작점 찾기: 설정한 ROI 바닥 범위 내에서 탐색
      l_min, l_max = int(np.min(left_pts[:, 0])), int(np.max(left_pts[:, 0]))
      r_min, r_max = int(np.min(right_pts[:, 0])), int(np.max(right_pts[:, 0]))


      l_min, l_max = max(0, l_min), min(self.width, l_max)
      r_min, r_max = max(0, r_min), min(self.width, r_max)


      histogram = np.sum(binary_img[int(self.height * 0.5):, :], axis=0)


      # [핵심 2] 점선 유실 시 대응 로직
      # 해당 ROI 범위 내에 픽셀이 하나도 없으면 argmax가 0을 반환하여 튀는 현상 방지
      if np.sum(histogram[l_min:l_max]) > 0:
          leftx_current = np.argmax(histogram[l_min:l_max]) + l_min
      else:
          # ROI 내에 픽셀이 없으면 ROI의 하단 중앙 지점을 임시 시작점으로 고정
          leftx_current = (l_min + l_max) // 2


      if np.sum(histogram[r_min:r_max]) > 0:
          rightx_current = np.argmax(histogram[r_min:r_max]) + r_min
      else:
          rightx_current = (r_min + r_max) // 2


      nonzero = binary_img.nonzero()
      nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
      left_lane_inds, right_lane_inds = [], []


      # [추가] 각 윈도우의 유효성 여부를 저장할 리스트
      valid_left_list = []
      valid_right_list = []


      # 윈도우가 한 칸 위로 갈 때 이동할 수 있는 최대 픽셀 수 (트랙 곡률에 따라 10~20 사이 조절)
      max_shift = 10


      for window in range(int(nwindows*0.5)):
          win_y_low = self.height - (window + 1) * window_height
          win_y_high = self.height - window * window_height


          win_xleft_low, win_xleft_high = leftx_current - margin, leftx_current + margin
          win_xright_low, win_xright_high = rightx_current - margin, rightx_current + margin


          # 시각화 (녹색 박스)
          cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 1)
          cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 1)


          good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                       (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
          good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                        (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]


          left_lane_inds.append(good_left)
          right_lane_inds.append(good_right)


          # [핵심 3] 수직 추적 관성 (Vertical Memory)
          # 픽셀이 없으면 이전 윈도우의 X 좌표를 그대로 유지하며 위로 올라감 (수직 계승)
          left_valid = len(good_left) > minpix
          right_valid = len(good_right) > minpix


          valid_left_list.append(left_valid)
          valid_right_list.append(right_valid)
          if left_valid:
              measured_left_x = int(np.mean(nonzerox[good_left]))
              # 급격한 이동 제한: $$leftx\_current = leftx\_current + \text{clip}(measured - current, -max, max)$$
              shift = measured_left_x - leftx_current
              leftx_current += np.clip(shift, -max_shift, max_shift)
          elif right_valid:
              # 왼쪽이 안 보이면 오른쪽을 기준으로 '강제 고정' (매우 중요!)
              leftx_current = rightx_current - self.lane_width


          if right_valid:
              measured_right_x = int(np.mean(nonzerox[good_right]))
              shift = measured_right_x - rightx_current
              rightx_current += np.clip(shift, -max_shift, max_shift)
          elif left_valid:
              # 오른쪽이 안 보이면 왼쪽을 기준으로 '강제 고정'
              rightx_current = leftx_current + self.lane_width


      # 데이터 병합 및 피팅 (오류 방지를 위해 픽셀 수 체크 추가)
      left_lane_inds = np.concatenate(left_lane_inds)
      right_lane_inds = np.concatenate(right_lane_inds)


      left_fit, right_fit = None, None
      # polyfit 에러('be poorly conditioned') 방지를 위해 충분한 점이 있을 때만 실행
      if len(left_lane_inds) > 100:
          left_fit = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
      if len(right_lane_inds) > 100:
          right_fit = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)


      return out_img, 0, left_fit, right_fit, valid_left_list, valid_right_list


  def sanity_check(self, left_fit, right_fit, min_dist=300, max_dist=520):
      """차선 간격 및 논리적 타당성 검사"""
      if left_fit is None or right_fit is None: return False
      y_eval = self.height - 1
      l_x = left_fit[0] * y_eval ** 2 + left_fit[1] * y_eval + left_fit[2]
      r_x = right_fit[0] * y_eval ** 2 + right_fit[1] * y_eval + right_fit[2]
      lane_width = r_x - l_x
      # 간격이 너무 좁거나 꼬였을 경우(l_x > r_x) 에러
      return min_dist < lane_width < max_dist


  # lib.py 파일의 LaneDetector 클래스 안에 추가하세요.
  def draw_lane_area(self, binary_img, left_fit, right_fit):
      """피팅된 곡선을 기반으로 차선 및 주행 영역을 채워서 시각화"""
      # 1. 그릴 도화지 생성 (컬러)
      out_img = np.dstack((binary_img, binary_img, binary_img)) * 0  # 검은색 배경


      # 2. 피팅이 정상적으로 되었을 때만 그리기
      if left_fit is None or right_fit is None:
          return out_img


      # 3. Y 좌표 및 피팅된 X 좌표 계산
      ploty = np.linspace(0, self.height - 1, self.height)
      left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
      right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]


      # 4. 다각형 좌표 생성
      # 왼쪽 곡선을 따라 내려가는 점들
      pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
      # 오른쪽 곡선을 따라 올라가는 점들 (순서를 뒤집어야 하나의 폐곡선이 됨)
      pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
      # 두 점들의 집합을 합쳐서 하나의 다각형 좌표 완성
      pts = np.hstack((pts_left, pts_right))


      # 5. 다각형 채우기 (가운데 주행 영역 - 초록색)
      cv2.fillPoly(out_img, np.int_([pts]), (0, 255, 0))


      # (선택) 왼쪽/오른쪽 차선 영역 자체를 좁게 채우고 싶다면?
      # margin = 20 # 차선 폭의 절반
      #
      # 왼쪽 차선 영역
      # pts_left_l = np.array([np.transpose(np.vstack([left_fitx - margin, ploty]))])
      # pts_left_r = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))])
      # pts_left_poly = np.hstack((pts_left_l, pts_left_r))
      # cv2.fillPoly(out_img, np.int_([pts_left_poly]), (255, 0, 0)) # 파란색
      #
      # 오른쪽 차선 영역도 동일한 방식으로 추가 가능


      return out_img


  def predict_lane(self, left_fit, right_fit, lane_width=320):
      """한쪽 차선만 보일 때 반대편 차선을 가상으로 생성"""
      # 1. 왼쪽만 있고 오른쪽이 없는 경우
      if left_fit is not None and right_fit is None:
          right_fit = np.copy(left_fit)
          right_fit[2] += lane_width # 상수항(C)에 폭을 더함


      # 2. 오른쪽만 있고 왼쪽이 없는 경우
      elif right_fit is not None and left_fit is None:
          left_fit = np.copy(right_fit)
          left_fit[2] -= lane_width # 상수항(C)에서 폭을 뺌


      return left_fit, right_fit




  def search_around_poly(self, binary_img, left_fit, right_fit, margin=50):
      """이전 프레임의 곡선 주변에서만 픽셀을 찾는 고속 모드"""
      nonzero = binary_img.nonzero()
      nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])


      left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] - margin)) &
                        (nonzerox < (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] + margin)))
      right_lane_inds = (
                  (nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] - margin)) &
                  (nonzerox < (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] + margin)))


      # 새로운 피팅 시도
      new_left_fit, new_right_fit = None, None
      try:
          if np.sum(left_lane_inds) > 500:  # 최소 픽셀 기준
              new_left_fit = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
          if np.sum(right_lane_inds) > 500:
              new_right_fit = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
      except:
          pass


      return new_left_fit, new_right_fit






class PurePursuitController:
  def calculate_steering(self, left_fit, right_fit, width, height, look_ahead=150):
      """오프셋 기반 Pure Pursuit 조향각 계산"""
      y_eval = height - 1
      if left_fit is not None and right_fit is not None:
          l_x = left_fit[0] * y_eval ** 2 + left_fit[1] * y_eval + left_fit[2]
          r_x = right_fit[0] * y_eval ** 2 + right_fit[1] * y_eval + right_fit[2]
          lane_center = (l_x + r_x) / 2
      else:
          lane_center = width / 2


      offset = lane_center - (width / 2)
      alpha = math.atan2(offset, look_ahead)
      # Steering formula: $$\delta = \arctan\left(\frac{2L \sin(\alpha)}{L_{fw}}\right)$$
      steering_angle = math.degrees(math.atan(2.0 * 1.0 * math.sin(alpha)))
      return steering_angle







