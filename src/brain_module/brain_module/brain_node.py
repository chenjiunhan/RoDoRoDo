import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # 基礎節點類
from std_msgs.msg import String  # ROS 2 字符串消息類型
import google.generativeai as genai
import os

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        # 聲明機器人名字參數
        self.declare_parameter('robot_name', 'default_robot')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # 動態訂閱主題名稱
        brain_input_topic = f'/{self.robot_name}/brain_input'
        brain_output_topic = f'/{self.robot_name}/brain_output'

        # 訂閱機器人專屬的 '/<robot_name>/brain_input' 主題
        self.subscription = self.create_subscription(
            String,
            brain_input_topic,
            self.think_callback,
            10)
        self.subscription  # 防止未使用警告

        genai.configure(api_key=os.environ['GOOGLE_GEMINI_API_KEY'])
        self.model = genai.GenerativeModel('gemini-1.5-flash')

        # 創建發布者，將思考結果發布到機器人專屬的 '/<robot_name>/brain_output' 主題
        self.publisher_ = self.create_publisher(String, brain_output_topic, 10)

        # 紀錄 log
        self.get_logger().info(f'{self.robot_name} BrainNode is up and running... Waiting for prompts.')

    def think_callback(self, msg):
        input_text = msg.data
        self.get_logger().info(f'Received prompt: "{input_text}"')

        # 執行 "思考" 邏輯（這裡用簡單的例子模擬）
        output_text = self.process_thought(input_text)

        # 創建要發布的消息
        output_msg = String()
        output_msg.data = output_text

        # 將結果發布到 '/<robot_name>/brain_output'
        self.publisher_.publish(output_msg)
        self.get_logger().info(f'Published thought: "{output_text}"')

    def process_thought(self, prompt):
        # 模擬思考邏輯，可以在這裡替換為更複雜的AI模型或算法
        # 例如：對prompt進行一些處理或調用外部AI服務

        try:
            response = self.model.generate_content(prompt)
            return response.text
        except Exception as e:
            self.get_logger().error(f"Failed to generate content: {e}")
            return ""


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy
    brain_node = BrainNode()  
    rclpy.spin(brain_node)  # 保持節點運行，直到手動停止
    brain_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)