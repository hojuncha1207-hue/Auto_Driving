import cv2
import AutonomousLibrary as al




def main():
   cap = cv2.VideoCapture("curv.mp4")
   env = al.Traffic_light_Detector()


   # 정지 기준 좌표 (화면 하단 끝에서 약 60픽셀 위, 상황에 맞춰 수정)
   STOP_LINE_LIMIT = 420


   while cap.isOpened():
       ret, frame = cap.read()
       if not ret: break


       img = cv2.resize(frame, (640, 480))
       H, W = img.shape[:2]


       # 신호등 ROI 설정 (신호등 위치에 따라 조정 필요)
       # 현재: 화면 상단 40%, 가로 중앙부
       tl_roi = img[0:int(H * 0.4), int(W * 0.2):int(W * 0.6)]


       # 신호등 인식 수행
       traffic, vis = env.object_detection(tl_roi)


       # 정지선 인식
       stop_y = env.get_stop_line(img)


       # 제어 로직(아두이노 통신하도록 수정 필요)
       speed = 100
       msg = "DRIVING"
       if traffic == "RED":
           if stop_y and stop_y > STOP_LINE_LIMIT:
               speed = 0
               msg = "STOP!!"
           else:
               speed = 30
               msg = "SLOW DOWN"


       # 시각화
       # 정지선 가이드라인 (Limit 표시)
       cv2.line(img, (0, STOP_LINE_LIMIT), (W, STOP_LINE_LIMIT), (255, 255, 0), 1)


       if stop_y:
           cv2.line(img, (0, stop_y), (W, stop_y), (0, 0, 255), 3)


       cv2.putText(img, f"LIGHT: {traffic}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
       cv2.putText(img, f"SPEED: {speed} | {msg}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


       # 신호등 ROI 영역 표시 (실제로 신호등이 박스 안에 들어오는지 확인용)
       cv2.rectangle(img, (int(W * 0.2), 0), (int(W * 0.6), int(H * 0.4)), (255, 0, 0), 2)


       cv2.imshow("frame", img)
       cv2.imshow("vis", vis)  # 이제 색상이 추출되면 여기에 나타납니다.


       if cv2.waitKey(20) & 0xFF == ord('q'):
           break


   cap.release()
   cv2.destroyAllWindows()




if __name__ == "__main__":
   main()
