import cv2
from ultralytics import YOLO

# YOLOv8 모델 로드
model = YOLO("test01_best.pt")
print(model.names)
cv2.namedWindow("aa")
# 웹캠 열기
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()


while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break
    
    # YOLOv8 객체 탐지 수행
    results = model(frame)
    print(f'cls = {int(results[0].boxes[0].cls)}')
    
    # 결과 이미지 가져오기
    annotated_frame = results[0].plot()
    
    # 화면에 표시
    cv2.imshow("aa", annotated_frame)
    #cv2.imshow("aa", frame)
    
    # 'q' 키를 누르면 종료
    i = cv2.waitKey(1)
    if i & 0xFF == ord('q'):
        break
    elif i & 0xFF == ord('c'):
        cv2.imwrite('sibal.jpg', annotated_frame)

# 자원 해제
cap.release()
cv2.destroyAllWindows()
