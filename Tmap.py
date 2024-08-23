from flask import Flask, request, jsonify
import sqlite3
import requests

app = Flask(__name__)

@app.route('/api/pothole', methods=['POST'])
def receive_pothole_data():
    data = request.json
    
    # 데이터 유효성 검사
    if 'latitude' not in data or 'longitude' not in data or 'pothole_size' not in data:
        return jsonify({'error': 'Invalid data'}), 400

    # 데이터베이스 저장
    save_to_database(data)

    # T맵 API로 지도에 포트홀 위치 표시
    display_on_tmap(data)

    # 웹훅을 통해 관리자에게 알림 전송
    notify_admin_via_webhook(data)

    return jsonify({'status': 'success'}), 200

def notify_admin_via_webhook(data):
    # 웹훅 URL 설정
    webhook_url = "https://your-webhook-url.com/notify"
    webhook_data = {
        "text": f"New pothole detected and displayed on Tmap: {data['latitude']}, {data['longitude']} (Size: {data['pothole_size']})"
    }
    
    # 웹훅으로 POST 요청 전송
    response = requests.post(webhook_url, json=webhook_data)
    
    if response.status_code == 200:
        print("Admin notified successfully.")
    else:
        print(f"Failed to notify admin: {response.status_code}, {response.text}")

def save_to_database(data):
    # 예: SQLite 데이터베이스에 포트홀 데이터를 저장
    conn = sqlite3.connect('pothole_data.db')
    c = conn.cursor()
    c.execute('''
        CREATE TABLE IF NOT EXISTS potholes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            latitude REAL,
            longitude REAL,
            altitude REAL,
            pothole_size TEXT,
            timestamp TEXT
        )
    ''')
    c.execute('''
        INSERT INTO potholes (latitude, longitude, altitude, pothole_size, timestamp)
        VALUES (?, ?, ?, ?, ?)
    ''', (data['latitude'], data['longitude'], data.get('altitude'), data['pothole_size'], data.get('timestamp')))
    conn.commit()
    conn.close()

def display_on_tmap(data):
    # T맵 API 사용 예시 (가상의 API 호출로 구현)
    tmap_url = "https://apis.openapi.sk.com/tmap/markers/pothole"
    headers = {
        "Content-Type": "application/json",
        "appKey": "your_tmap_api_key"
    }
    tmap_data = {
        "version": 1,
        "markers": [{
            "lat": data['latitude'],
            "lon": data['longitude'],
            "size": "small",  # "size"는 포트홀의 크기에 따라 다르게 설정 가능
            "type": "Pothole",
            "alt": data.get('altitude', 0),
            "label": f"Pothole {data['pothole_size']}"
        }]
    }

    response = requests.post(tmap_url, headers=headers, json=tmap_data)
    if response.status_code == 200:
        print("Pothole successfully displayed on Tmap")
    else:
        print(f"Failed to display pothole on Tmap: {response.status_code}, {response.text}")

if __name__ == '__main__':
    app.run(debug=True)
