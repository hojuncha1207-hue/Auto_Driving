
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

      self.lane_width = LANE_WIDTH_PIXELS


  def mask_green_floor(self, img):
      """�ʷϻ� �ٴ� ������ ���������� ����ŷ"""


      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      lower_green = np.array([35, 35, 35])  # (H: 35->30, S: 50->20, V: 50->20)
      upper_green = np.array([85, 255, 255])  # (H: 85->100)


      # 3. ����ũ ���� (�ʷϻ� ������ ����� �̹���)
      green_mask = cv2.inRange(hsv, lower_green, upper_green)
      # green_mask ���� ���Ŀ� �߰�
      kernel = np.ones((3, 3), np.uint8)
      # ����ũ ������ ���� ���۵��� �޿�� ��踦 �ε巴�� ��
      green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
      green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_DILATE, kernel)  # ��¦ �� �а� �����

      masked_img = img.copy()

      masked_img[green_mask != 0] = [0, 0, 0]


      return masked_img ,green_mask


  def erase_right_of_green(self, img, green_mask):

      h, w = green_mask.shape
      result_img = img.copy()

      found_green = np.any(green_mask > 0, axis=1)
      first_green_indices = np.argmax(green_mask > 0, axis=1)


      for y in range(h):
          if found_green[y]:
              x_start = first_green_indices[y]+3
              result_img[y, x_start:] = [0, 0, 0]


      return result_img


  def get_bev(self, frame, gap_top, height_top, x_offset):
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
      BEV �̹����� �ϴ� �� �ﰢ�� ������ ���� �ȼ��� ���� Ȯ���Ͽ� ä��
      y_start: Ȯ���� ������ ���� (���� �ﰢ���� ���۵Ǵ� ����)
      """
      h, w = bev_img.shape[:2]
      filled_img = bev_img.copy()


      # y_start �������� �� �Ʒ����� �� �پ� �˻�
      for y in range(y_start, h):
          # ���� ��(y)���� 0�� �κ�(������)�� ã�� �� ��(y-1)�� ������ �����
          # binary �̹����� ���� �÷� �̹����� ��� ��� ���� �����ϵ��� ó��
          if len(filled_img.shape) == 3:  # �÷� �̹��� (3ä��)
              mask = np.all(filled_img[y] == 0, axis=-1)
              filled_img[y][mask] = filled_img[y - 1][mask]
          else:  # ���� �̹��� (1ä��)
              mask = (filled_img[y] == 0)
              filled_img[y][mask] = filled_img[y - 1][mask]


      return filled_img


  def detect_line_type(self, valid_windows_list, min_ratio=0.6):
      """
      �� �������� ��ȿ�� ������ �������� ���� Ÿ�� �Ǻ�
      valid_windows_list: [True, True, False, True, ...] ������ ����Ʈ
      min_ratio: �Ǽ����� �Ǵ��� �ּ� ���� (��: 0.6 = 60% �̻�)
      """
      if not valid_windows_list:
          return "Lost"


      # ��ȿ��(True) �������� ������ ���ϴ�.
      num_valid = sum(valid_windows_list)
      total_windows = len(valid_windows_list)


      # ���� ���
      fill_ratio = num_valid / total_windows if total_windows > 0 else 0


      # ������ ���� Ÿ�� ���� (Ʃ�� �ʿ�)
      if fill_ratio > min_ratio:
          return "Solid"  # �Ǽ�
      elif fill_ratio > 0.2: # �ּ����� ������ �Ѿ�� �������� ����
          return "Dashed" # ����
      else:
          return "Lost"   # ��ħ


  def extend_pixels_side_only(self, binary_img, y_start=380, center_gap=100):
      """
      �߾� �κ��� �����ϰ� ��/���� �ϴ� �� ������ ���� Ȯ����.
      center_gap: �߾ӿ��� ������ ������ ��ü ���� �� (�ȼ� ����)
      """
      h, w = binary_img.shape[:2]
      filled_img = binary_img.copy()


      # �߾� ���ܸ� ���� ��輱 ���
      left_boundary = (w // 2) - (center_gap // 2)
      right_boundary = (w // 2) + (center_gap // 2)


      for y in range(y_start, h):
          # 1. ���� �ٿ��� ���� 0(����)�� ����ũ ����
          zero_mask = (filled_img[y] == 0)


          # 2. �߾� ����(left_boundary ~ right_boundary)�� ����ũ���� ���� (False ó��)
          # ��, �߾� �κ��� 0�̾ ������ �����ؿ��� ����
          zero_mask[left_boundary:right_boundary] = False


          # 3. ����ŷ�� ����(��/�� ���̵�)�� ���� �� ����
          filled_img[y][zero_mask] = filled_img[y-1][zero_mask]


      return filled_img


  def get_binary_hls(self, img, gap_top=GAP_TOP, height_top=HEIGHT_TOP, x_offset=X_OFFSET, threshold=160):
      """HLS ���͸��� Canny Edge�� �����Ͽ� ���� ����"""


      # 1. ���� �̹��� BEV ��ȯ (Canny��)
      orbev_img, _ = self.get_bev(img, gap_top, height_top, x_offset)


      # 2. HLS ���͸� (���� �̹��� ����)
      hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
      white_lane_binary = np.zeros_like(hls[:, :, 1])
      # ���(L)�� ���� ä��(S)�� ���� ���� ���� (��� ����)
      white_lane_binary[(hls[:, :, 1] > 150)] = 255


      # 3. ���� ���͸��� �̹��� BEV ��ȯ
      bev_binary, _ = self.get_bev(white_lane_binary, gap_top, height_top, x_offset)


      # 4. Canny Edge ���� (�̹� ��ȯ�� BEV �÷� �̹��� ���)
      gray_bev = cv2.cvtColor(orbev_img, cv2.COLOR_BGR2GRAY)
      blurred = cv2.GaussianBlur(gray_bev, (5, 5), 0)
      canny_edge = cv2.Canny(blurred, 50, 150)


      # 5. ���� ����(bev_binary) + ���� ����(canny_edge) ����
      combined = cv2.bitwise_or(bev_binary, canny_edge)


      return combined


  def block_filter(self, binary_img, nwindows=40, min_w=5, max_w=9):
      """���簢�� ��� ������ �ɰ��� ���� �β�(�ʺ�)�� ���������� ��ü ����"""
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


          # 1. ���μ� ����: ���ΰ� ���κ��� ���� �� (���� 0.7 �̻�)
          aspect_ratio = h / w
          if aspect_ratio < 0.2: continue  # �α赵 ���� �??


          # 2. ���� ���� ��ȭ: �����̳� �ָ� �ִ� ������ 300���� ���� �� ����
          if area < 50:
              continue


          # 3. �ִ� �β� ���� ���� �Ǵ� Ȯ��: Ŀ�꿡�� ��ģ ���� ����
          if w < 5:
              continue


          filtered_img[labels == i] = 255
      return filtered_img


  def apply_dual_roi_mask(self, binary_img, left_pts, right_pts):
      """���� �̹����� �¿� ���� ROI�� �����Ͽ� ���� ������ ����"""
      mask = np.zeros_like(binary_img)
      # ���� ROI ���� ���� (��� 255)
      cv2.fillPoly(mask, [np.int32(left_pts)], 255)
      # ���� ROI ���� ���� (��� 255)
      cv2.fillPoly(mask, [np.int32(right_pts)], 255)


      # ���� ���� �̹����� ����ũ �ռ�
      return cv2.bitwise_and(binary_img, mask)


  def draw_dual_roi(self, img, left_pts, right_pts):
      """���� ���� �¿� ROI �׵θ��� �ð�ȭ"""
      draw_img = img.copy() # ���� ������ ���� ���纻 ���
      # ������ ������(Red), �������� �Ķ���(Blue) �׵θ� �׸���
      cv2.polylines(draw_img, [np.int32(left_pts)], True, (0, 0, 255), 3)
      cv2.polylines(draw_img, [np.int32(right_pts)], True, (255, 0, 0), 3)
      return draw_img


  def sliding_window(self, binary_img, left_pts, right_pts, nwindows=12, margin=60, minpix=50):
      """���� ������ ��ġ�� �������� �ٷ� �� ������ Ž���ϴ� ���� ���� ���"""
      out_img = np.dstack((binary_img, binary_img, binary_img))
      window_height = int(self.height // nwindows)


      # [1] ������ ã��: ������ ROI �ٴ� ���� ������ Ž��
      l_min, l_max = int(np.min(left_pts[:, 0])), int(np.max(left_pts[:, 0]))
      r_min, r_max = int(np.min(right_pts[:, 0])), int(np.max(right_pts[:, 0]))


      l_min, l_max = max(0, l_min), min(self.width, l_max)
      r_min, r_max = max(0, r_min), min(self.width, r_max)


      histogram = np.sum(binary_img[int(self.height * 0.5):, :], axis=0)


      # [�ٽ� 2] ���� ���� �� ���� ����
      # �ش� ROI ���� ���� �ȼ��� �ϳ��� ������ argmax�� 0�� ��ȯ�Ͽ� Ƣ�� ���� ����
      if np.sum(histogram[l_min:l_max]) > 0:
          leftx_current = np.argmax(histogram[l_min:l_max]) + l_min
      else:
          # ROI ���� �ȼ��� ������ ROI�� �ϴ� �߾� ������ �ӽ� ���������� ����
          leftx_current = (l_min + l_max) // 2


      if np.sum(histogram[r_min:r_max]) > 0:
          rightx_current = np.argmax(histogram[r_min:r_max]) + r_min
      else:
          rightx_current = (r_min + r_max) // 2


      nonzero = binary_img.nonzero()
      nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
      left_lane_inds, right_lane_inds = [], []


      # [�߰�] �� �������� ��ȿ�� ���θ� ������ ����Ʈ
      valid_left_list = []
      valid_right_list = []


      # �����찡 �� ĭ ���� �� �� �̵��� �� �ִ� �ִ� �ȼ� �� (Ʈ�� ����� ���� 10~20 ���� ����)
      max_shift = 10


      for window in range(int(nwindows*0.5)):
          win_y_low = self.height - (window + 1) * window_height
          win_y_high = self.height - window * window_height


          win_xleft_low, win_xleft_high = leftx_current - margin, leftx_current + margin
          win_xright_low, win_xright_high = rightx_current - margin, rightx_current + margin


          # �ð�ȭ (��� �ڽ�)
          cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 1)
          cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 1)


          good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                       (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
          good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                        (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]


          left_lane_inds.append(good_left)
          right_lane_inds.append(good_right)


          # [�ٽ� 3] ���� ���� ���� (Vertical Memory)
          # �ȼ��� ������ ���� �������� X ��ǥ�� �״�� �����ϸ� ���� �ö� (���� ���)
          left_valid = len(good_left) > minpix
          right_valid = len(good_right) > minpix


          valid_left_list.append(left_valid)
          valid_right_list.append(right_valid)
          if left_valid:
              measured_left_x = int(np.mean(nonzerox[good_left]))
              # �ް��� �̵� ����: $$leftx\_current = leftx\_current + \text{clip}(measured - current, -max, max)$$
              shift = measured_left_x - leftx_current
              leftx_current += np.clip(shift, -max_shift, max_shift)
          elif right_valid:
              # ������ �� ���̸� �������� �������� '���� ����' (�ſ� �߿�!)
              leftx_current = rightx_current - self.lane_width


          if right_valid:
              measured_right_x = int(np.mean(nonzerox[good_right]))
              shift = measured_right_x - rightx_current
              rightx_current += np.clip(shift, -max_shift, max_shift)
          elif left_valid:
              # �������� �� ���̸� ������ �������� '���� ����'
              rightx_current = leftx_current + self.lane_width


      # ������ ���� �� ���� (���� ������ ���� �ȼ� �� üũ �߰�)
      left_lane_inds = np.concatenate(left_lane_inds)
      right_lane_inds = np.concatenate(right_lane_inds)


      left_fit, right_fit = None, None
      # polyfit ����('be poorly conditioned') ������ ���� ����� ���� ���� ���� ����
      if len(left_lane_inds) > 100:
          left_fit = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
      if len(right_lane_inds) > 100:
          right_fit = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)


      return out_img, 0, left_fit, right_fit, valid_left_list, valid_right_list


  def sanity_check(self, left_fit, right_fit, min_dist=300, max_dist=520):
      """���� ���� �� ���� Ÿ�缺 �˻�"""
      if left_fit is None or right_fit is None: return False
      y_eval = self.height - 1
      l_x = left_fit[0] * y_eval ** 2 + left_fit[1] * y_eval + left_fit[2]
      r_x = right_fit[0] * y_eval ** 2 + right_fit[1] * y_eval + right_fit[2]
      lane_width = r_x - l_x
      # ������ �ʹ� ���ų� ������ ���(l_x > r_x) ����
      return min_dist < lane_width < max_dist


  # lib.py ������ LaneDetector Ŭ���� �ȿ� �߰��ϼ���.
  def draw_lane_area(self, binary_img, left_fit, right_fit):
      """���õ� ��� ������� ���� �� ���� ������ ä���� �ð�ȭ"""
      # 1. �׸� ��ȭ�� ���� (�÷�)
      out_img = np.dstack((binary_img, binary_img, binary_img)) * 0  # ������ ���


      # 2. ������ ���������� �Ǿ��� ���� �׸���
      if left_fit is None or right_fit is None:
          return out_img


      # 3. Y ��ǥ �� ���õ� X ��ǥ ���
      ploty = np.linspace(0, self.height - 1, self.height)
      left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
      right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]


      # 4. �ٰ��� ��ǥ ����
      # ���� ��� ���� �������� ����
      pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
      # ������ ��� ���� �ö󰡴� ���� (������ ������� �ϳ��� ���� ��)
      pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
      # �� ������ ������ ���ļ� �ϳ��� �ٰ��� ��ǥ �ϼ�
      pts = np.hstack((pts_left, pts_right))


      # 5. �ٰ��� ä��� (��� ���� ���� - �ʷϻ�)
      cv2.fillPoly(out_img, np.int_([pts]), (0, 255, 0))


      # (����) ����/������ ���� ���� ��ü�� ���� ä��� �ʹٸ�?
      # margin = 20 # ���� ���� ����
      #
      # ���� ���� ����
      # pts_left_l = np.array([np.transpose(np.vstack([left_fitx - margin, ploty]))])
      # pts_left_r = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))])
      # pts_left_poly = np.hstack((pts_left_l, pts_left_r))
      # cv2.fillPoly(out_img, np.int_([pts_left_poly]), (255, 0, 0)) # �Ķ���
      #
      # ������ ���� ������ ������ ������� �߰� ����


      return out_img


  def predict_lane(self, left_fit, right_fit, lane_width=320):
      """���� ������ ���� �� �ݴ��� ������ �������� ����"""
      # 1. ���ʸ� �ְ� �������� ���� ���
      if left_fit is not None and right_fit is None:
          right_fit = np.copy(left_fit)
          right_fit[2] += lane_width # �����(C)�� ���� ����


      # 2. �����ʸ� �ְ� ������ ���� ���
      elif right_fit is not None and left_fit is None:
          left_fit = np.copy(right_fit)
          left_fit[2] -= lane_width # �����(C)���� ���� ��


      return left_fit, right_fit




  def search_around_poly(self, binary_img, left_fit, right_fit, margin=50):
      """���� �������� � �ֺ������� �ȼ��� ã�� ��� ���"""
      nonzero = binary_img.nonzero()
      nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])


      left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] - margin)) &
                        (nonzerox < (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] + margin)))
      right_lane_inds = (
                  (nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] - margin)) &
                  (nonzerox < (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] + margin)))


      # ���ο� ���� �õ�
      new_left_fit, new_right_fit = None, None
      try:
          if np.sum(left_lane_inds) > 500:  # �ּ� �ȼ� ����
              new_left_fit = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
          if np.sum(right_lane_inds) > 500:
              new_right_fit = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
      except:
          pass


      return new_left_fit, new_right_fit






class PurePursuitController:
  def calculate_steering(self, left_fit, right_fit, width, height, look_ahead=150):
      """������ ��� Pure Pursuit ���Ⱒ ���"""
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







