import cv2
import mysql.connector
import readsrv as srv
from ultralytics import YOLO

# ✅ MySQL 연결 설정
conn = mysql.connector.connect(
    host="127.0.0.1",
    user="root",
    password="red79166",
    database="demo"
)
cursor = conn.cursor()

# ✅ YOLO 모델 및 비디오 설정
model_path = "/media/park/33DF49D6718AD56F/runs/detect/good/weights/best.pt"
model = YOLO(model_path)  # YOLO 모델 로드
video_path = "/home/park/문서/DCIM (1)/DJI_202501241608_008/DJI_20250124161811_0010_V.MP4"

# ✅ 비디오 캡처
cap = cv2.VideoCapture(video_path)

# ✅ 비디오 속성 가져오기
fps = int(cap.get(cv2.CAP_PROP_FPS))  # 초당 프레임 수
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # 전체 프레임 수
duration = frame_count / fps  # 비디오 전체 길이(초)
print(f"비디오 길이: {duration:.2f}초")

# ✅ 탐지 데이터 저장을 위한 리스트
data = []
last_detected_time = 0
previous_time = 0
frame_index = 0  # 프레임 카운트

# ✅ GPS 타임스탬프 정렬
timestamp_list = sorted(srv.df.iloc[:, 0].tolist())  # 오름차순 정렬

def binary_search_closest(arr, x):
    """이진 탐색을 이용해 가장 가까운 값을 찾음"""
    if not arr:
        return None  # 빈 리스트 예외 처리

    left, right = 0, len(arr) - 1
    while left < right:
        mid = (left + right) // 2
        if arr[mid] < x:
            left = mid + 1
        else:
            right = mid

    # 가장 가까운 값 반환
    if left == 0:
        return arr[0]
    if left >= len(arr):
        return arr[-1]

    before = arr[left - 1]
    after = arr[left]
    return before if abs(before - x) <= abs(after - x) else after

def search_gps(timestamp_sec):
    """주어진 타임스탬프에 가장 가까운 GPS 데이터 찾기"""
    global last_detected_time

    index = binary_search_closest(timestamp_list, timestamp_sec)
    if index is None:
        return None  # 검색 결과 없음

    # 이전 감지와 차이가 0.1초 이하이면 무시
    if index - last_detected_time <= 5:
        return None

    last_detected_time = index  # 최신 감지 시간 업데이트

    # 가장 가까운 시간에 해당하는 GPS 데이터 검색
    matched_row = srv.df[srv.df.iloc[:, 0] == index]
    
    if not matched_row.empty:
        latitude = matched_row.iloc[0, 1]  # 위도
        longitude = matched_row.iloc[0, 2]  # 경도
        return latitude, longitude, index
    else:
        return None

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break  # 더 이상 프레임이 없으면 종료

    # ✅ 현재 프레임의 타임스탬프 (초 단위)
    timestamp_sec = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000
    estimated_timestamp = frame_index / fps  # 보정된 타임스탬프

    # ✅ YOLO 감지 수행
    results = model(frame)

    # ✅ 탐지 결과 바운딩 박스 그리기
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
            confidence = box.conf[0]  # 신뢰도 점수
            class_id = int(box.cls[0])  # 클래스 ID
            label = f"{model.names[class_id]} {confidence:.2f}"  # 라벨 텍스트

            # 바운딩 박스 그리기 (초록색, 두께 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # 텍스트 라벨 추가 (노란색, 두께 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # ✅ GPS 데이터 검색 및 MySQL 저장
    search_result = search_gps(timestamp_sec)
    if search_result is not None:
        latitude, longitude, current_time = search_result
        if current_time - previous_time > 5:
            data.append([current_time, latitude, longitude])
            previous_time = current_time  # 최신 감지 시간 업데이트

            # MySQL 데이터베이스에 저장
            insert_sql = "INSERT INTO location_data (latitude, longitude) VALUES (%s, %s)"
            cursor.execute(insert_sql, (latitude, longitude))
            conn.commit()

    # ✅ 진행 상황 출력
    print(f"Processing frame {frame_index}/{frame_count}...", end="\r")

    frame_index += 1  # 프레임 인덱스 업데이트

cap.release()
cv2.destroyAllWindows()

# ✅ `region` 필드 자동 업데이트
update_sql = """
UPDATE location_data AS l
SET l.region = (
    SELECT a.name
    FROM administrative_areas AS a
    WHERE ST_Contains(a.geom, ST_GeomFromText(CONCAT('POINT(', l.latitude, ' ', l.longitude, ')'), 4326))
    ORDER BY ST_Area(a.geom) ASC
    LIMIT 1
)
WHERE l.region IS NULL
"""
cursor.execute(update_sql)
conn.commit()

# ✅ 저장된 데이터 확인
check_sql = "SELECT * FROM location_data"
cursor.execute(check_sql)
result = cursor.fetchall()

print("\n📌 감지된 데이터 (Timestamp, Latitude, Longitude, Region):")
for row in result:
    print(row)

# ✅ MySQL 연결 종료
cursor.close()
conn.close()
