import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # 적절한 메시지 타입을 임포트
import numpy as np
import requests
import json

class PotholeDetector(Node):
    def __init__(self):
        super().__init__('pothole_detector_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.sensor_callback,
            10
        )

    def sensor_callback(self, msg):
        sensor_data = np.array(msg.data)
        if self.detect_pothole(sensor_data):
            pothole_info = {
                "latitude": sensor_data[0],  # sensor_data에서 위도 값 가져오기
                "longitude": sensor_data[1],  # sensor_data에서 경도 값 가져오기
                "altitude": sensor_data[2],  # sensor_data에서 고도 값 가져오기
                "pothole_size": "medium"  # 이 부분은 필요에 따라 계산하거나 추가로 설정 가능
            }
            try:
                # AWS API Gateway에 HTTP POST 요청 전송
                response = requests.post("https://your-api-gateway.amazonaws.com/pothole", json=pothole_info)
                response.raise_for_status()  # 잘못된 HTTP 상태 코드를 예외로 처리
                self.get_logger().info(f"Pothole detected and sent to AWS: {pothole_info}, Response: {response.status_code}")
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f"Failed to send data to AWS: {str(e)}")

    def detect_pothole(self, sensor_data):
        threshold = 2.5
        return np.any(sensor_data > threshold)

def main(args=None):
    rclpy.init(args=args)
    node = PotholeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

