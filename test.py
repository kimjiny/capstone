from flask import Flask, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)  # CORS 설정 (프론트엔드에서 API를 호출할 수 있도록 허용)

# 포트홀 데이터 (예제 데이터)
potholes = [
    {
        "id": 1,
        "location": "서울특별시 강남구 테헤란로",
        "latitude": 37.498095,
        "longitude": 127.02761,
        "status": "처리중",
    },
    {
        "id": 2,
        "location": "서울특별시 서초구 서초대로",
        "latitude": 37.493923,
        "longitude": 127.014656,
        "status": "미완료",
    },
    {
        "id": 3,
        "location": "부산광역시 해운대구",
        "latitude": 35.163147,
        "longitude": 129.163636,
        "status": "처리완료",
    },
]

@app.route("/potholes", methods=["GET"])
def get_potholes():
    return jsonify(potholes)  # JSON 데이터 반환

if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=3000)
