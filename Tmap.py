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

    webhook_url = "https://discord.com/api/webhooks/1279466397759438929/M8sRC8Cf01LJQMLNTvRM2oBYCFMVm3xG_4M-TX2on3jG5HY4p4vkG6WZS5mF9d3yzmwG"  # 앞에서 복사한 웹훅 URL을 여기에 붙여넣기
    # 웹훅을 통해 관리자에게 알림 전송
    send_discord_message(data)

    return jsonify({'status': 'success'}), 200

def send_discord_message(webhook_url, data):
    
    data = {
        "content": f"New pothole detected and displayed on Tmap: {data['latitude']}, {data['longitude']} (Size: {data['pothole_size']})",  # 보낼 메시지 내용
        "username": "Drone Bot"  # 디스코드에 표시될 봇 이름
    }

    result = requests.post(webhook_url, json=data)

    if result.status_code == 200:
        print("Admin notified successfully.")
    else:
        print(f"Failed to notify admin: {result.status_code}, {result.content}")


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
