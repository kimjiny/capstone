import json
import requests

def lambda_handler(event, context):
    # MQTT로부터 들어온 메시지 파싱
    message = json.loads(event['Records'][0]['Sns']['Message'])
    
    # 앱 서버에 전송할 데이터 준비
    pothole_data = {
        "latitude": message.get("latitude"),
        "longitude": message.get("longitude"),
        "altitude": message.get("altitude"),
        "pothole_size": message.get("pothole_size"),
        "timestamp": message.get("timestamp"),
    }

    # 앱 서버의 엔드포인트로 POST 요청 전송
    response = requests.post("https://your-app-server.com/api/pothole", json=pothole_data)
    
    # 앱 서버로의 전송 결과를 로그로 출력
    if response.status_code == 200:
        print("Pothole data successfully sent to the app server.")
    else:
        print(f"Failed to send pothole data: {response.status_code}, {response.text}")
    
    return {
        'statusCode': 200,
        'body': json.dumps('Process completed')
    }
