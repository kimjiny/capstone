import cv2
from ultralytics import YOLO
model = YOLO('yolov8l.pt') 

video_path = 'video.mp4'  # 동영상 파일 경로
cap = cv2.VideoCapture(video_path)  # 비디오 파일

# 저장할 프레임 카운터 초기화
frame_counter = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("동영상 종료 또는 읽기 실패")
        break

    # YOLO 모델로 객체 감지
    results = model(frame)

    # 감지된 객체가 있으면 프레임 저장
    if len(results[0].boxes) > 0:  # 감지된 객체가 있는지 확인
        frame_counter += 1
        frame_filename = f"frame_{frame_counter}.jpg"
        cv2.imwrite(frame_filename, frame)  # 프레임 저장
        print(f"Saved frame: {frame_filename}")

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
