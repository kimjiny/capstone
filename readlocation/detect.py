import cv2
import time
import readsrv as srv
from ultralytics import YOLO
# model_path = "yolo_path"
# model = YOLO(yolov8n.pt)
# video_path = "video_path"
# cap = cv2.VideoCapture(video_path)

# fps = cap.get(cv2.CAP_PROP_FPS)  # 초당 프레임 수
# frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # 전체 프레임 수
# duration = frame_count / fps  # 비디오 전체 길이(초)
# print(f"비디오 길이: {duration:.2f}초")
# detected_time = []


def binary_search_closest(arr, x):
    """이진 분류 값이랑 제일 가까운 값 찾기"""
    if not arr:
        return None  # 빈 리스트 예외 처리

    left, right = 0, len(arr) - 1

    while left < right:
        mid = (left + right) // 2

        if arr[mid] < x:
            left = mid + 1
        else:
            right = mid

    if left == 0:
        return arr[0]
    if left == len(arr):
        return arr[-1]

    before = arr[left - 1]
    after = arr[left]

    return before if abs(before - x) <= abs(after - x) else after
# print(len(srv.df.iloc[:,0].tolist()))
index = binary_search_closest(srv.df.iloc[:,0].tolist(),0.1)
print(index)
print(srv.df[srv.df[0]==index])


# while cap.isOpened():
#     ret, frame = cap.read()
#     if not ret:
#         break
#     # 현재 프레임의 타임스탬프(ms)
#     timestamp_ms = cap.get(cv2.CAP_PROP_POS_MSEC)  # 밀리초 단위
#     timestamp_sec = timestamp_ms / 1000  # 초 단위 변환

#     # YOLO 감지 수행
#     results = model(frame)

#     # 객체가 감지되었을 때만 타임스탬프 출력
#     if len(results[0].boxes) > 0:
#         print(f"Detect at {timestamp_sec:.2f} seconds")
#         detected_time.append(timestamp_sec)
        

#     # 프레임 출력
#     cv2.imshow("YOLO Detection", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
