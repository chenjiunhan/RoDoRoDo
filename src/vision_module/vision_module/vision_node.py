import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 可以根據需要替換為更合適的消息類型
# 如果你需要使用OpenCV進行圖像處理，可以導入opencv-python
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # 聲明機器人名字參數
        self.declare_parameter('robot_name', 'default_robot')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # 動態設置與機器人交互的主題名稱
        vision_input_topic = f'/{self.robot_name}/vision_input'
        vision_output_topic = f'/{self.robot_name}/vision_output'

        # 訂閱 '/<robot_name>/vision_input' 主題，接收圖像數據
        self.subscription = self.create_subscription(
            String,  # 這裡可以使用適合的圖像消息類型，如 sensor_msgs/Image
            vision_input_topic,
            self.image_callback,
            10)
        self.subscription  # 防止未使用警告

        # 創建發布者，將視覺處理結果發布到 '/<robot_name>/vision_output' 主題
        self.publisher_ = self.create_publisher(String, vision_output_topic, 10)

        self.get_logger().info(f'{self.robot_name} VisionNode is up and running... Waiting for images.')

    def image_callback(self, msg):
        # 在這裡處理圖像數據，這是圖像處理的核心部分
        self.get_logger().info(f'Received image data for {self.robot_name}')

        # 模擬圖像處理邏輯
        result = self.process_image(msg.data)  # 替換為實際的圖像處理代碼

        # 將處理結果發布到 '/<robot_name>/vision_output' 主題
        output_msg = String()
        output_msg.data = result
        self.publisher_.publish(output_msg)
        self.get_logger().info(f'Published vision result: {result}')

    def process_image(self, image_data):
        # 假設這裡是圖像處理代碼
        # 例如，使用 OpenCV 進行處理
        # img = cv2.imdecode(np.frombuffer(image_data, np.uint8), cv2.IMREAD_COLOR)
        # ...進行圖像處理...
        self.get_logger().info('Processing image...')

        # 模擬返回的處理結果
        return "Processed Image Result"

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)