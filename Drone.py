import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # 적절한 메시지 타입을 임포트
import numpy as np
import requests
import json
import time

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
                "pothole_size": sensor_data[3],  #sensor_data에서 포트홀 사이즈 값 가져오기 0 -> 작은 사이즈 | 1 -> 중간사이즈 | 2 -> 큰 사이즈
                "timestamp" : time.time()
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
        return (sensor_data[4] > threshold) #sensor_data[4]에 인공지능이 계산한 값 저장. 만약에 threshold보다 값이 크면 True를 반환.

def main(args=None):
    rclpy.init(args=args)
    node = PotholeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

