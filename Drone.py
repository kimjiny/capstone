import rclpy
from rclpy.node import Node
import numpy as np
import requests
import json

class PotholeDetector(Node):
    def __init__(self):
        super().__init__('pothole_detector_node')
        self.subscription = self.create_subscription('sensor_data', self.sensor_callback, 10)

    def sensor_callback(self, msg):
        sensor_data = np.array(msg.data)
        if self.detect_pothole(sensor_data):
            pothole_info = {
                "latitude": 37.5665,
                "longitude": 126.9780,
                "altitude": 150.0,
                "pothole_size": "medium"
            }
            # AWS API Gateway에 HTTP POST 요청 전송
            response = requests.post("https://your-api-gateway.amazonaws.com/pothole", json=pothole_info)
            self.get_logger().info(f"Pothole detected and sent to AWS: {pothole_info}, Response: {response.status_code}")

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
