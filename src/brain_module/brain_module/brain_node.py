import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # 基礎節點類
from std_msgs.msg import String  # ROS 2 字符串消息類型
import google.generativeai as genai
import os

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')  # 初始化節點，名稱為 'brain_node'
        
        # 訂閱 '/brain_input' 主題
        self.subscription = self.create_subscription(
            String,
            'brain_input',
            self.think_callback,
            10)
        self.subscription  # 防止未使用警告

        genai.configure(api_key=os.environ['GOOGLE_GEMINI_API_KEY'])
        self.model = genai.GenerativeModel('gemini-1.5-flash')

        # 創建發布者，將思考結果發布到 '/brain_output' 主題
        self.publisher_ = self.create_publisher(String, 'brain_output', 10)

        # 紀錄 log
        self.get_logger().info('BrainNode is up and running... Waiting for prompts.')

    def think_callback(self, msg):
        input_text = msg.data
        self.get_logger().info(f'Received prompt: "{input_text}"')

        # 執行 "思考" 邏輯（這裡用簡單的例子模擬）
        output_text = self.process_thought(input_text)

        # 創建要發布的消息
        output_msg = String()
        output_msg.data = output_text

        # 將結果發布到 '/brain_output'
        self.publisher_.publish(output_msg)
        self.get_logger().info(f'Published thought: "{output_text}"')

    def process_thought(self, prompt):
        # 模擬思考邏輯，可以在這裡替換為更複雜的AI模型或算法
        # 例如：對prompt進行一些處理或調用外部AI服務
        
        prompt_prefix = str([]) + ", 以上是你的記憶，記憶是過去的事情。你現在是一隻狗，並且正在思考與自言自語，同時你可能正在進行一些行動，你說的話會是下一步的行動，請自行發揮想像力世界的變化，並且有時間的推進。"
        
        final_prompt = prompt_prefix
        
        if prompt:
            final_prompt += "請根據以下提示詞，簡短回應你的行動，說話的對象是你自己：" + prompt
        else:
            final_prompt += "簡短回應你的行動，說話的對象是你自己。"

        #self.logger.debug(f"Prompt: {final_prompt}")

        try:
            response = self.model.generate_content(final_prompt)
            return response.text
        except Exception as e:
            return ""


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy

    brain_node = BrainNode()  # 創建節點

    rclpy.spin(brain_node)  # 保持節點運行，直到手動停止

    # 清理和關閉
    brain_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()