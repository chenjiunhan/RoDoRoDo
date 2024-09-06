from rclpy.node import Node
from std_msgs.msg import String
from common_interface.srv import Register
from sensor_msgs.msg import Image as RosImage  # 使用ROS Image消息類型
import random
import rclpy
import time

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')  # 初始化節點，名稱基於機器人名字

        # 聲明機器人名字參數
        self.declare_parameter('robot_name', 'default_robot')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # 動態設置與brain_module、vision_module和memory_module交互的主題名稱
        brain_input_topic = f'/{self.robot_name}/brain_input'
        brain_output_topic = f'/{self.robot_name}/brain_output'
        vision_input_topic = f'/{self.robot_name}/vision_input'
        vision_output_topic = f'/{self.robot_name}/vision_output'
        store_memory_topic = f'/{self.robot_name}/store_memory'
        retrieve_memory_topic = f'/{self.robot_name}/retrieve_memory'
        memory_output_topic = f'/{self.robot_name}/memory_output'
        desktop_world_topic = f'/desktop_world/{self.robot_name}/world_state'

        self.memory = ""  # 用於存儲檢索到的記憶

        # 訂閱來自brain_module的消息
        self.brain_subscription = self.create_subscription(
            String,
            brain_output_topic,
            self.brain_callback,
            10)

        # 訂閱來自vision_module的消息
        self.vision_subscription = self.create_subscription(
            String,
            vision_output_topic,
            self.vision_callback,
            10)

        # 訂閱來自memory_module的記憶檢索結果
        self.memory_subscription = self.create_subscription(
            String,
            memory_output_topic,
            self.memory_callback,
            10)

        # 創建發布者，將控制命令發送到brain_module
        self.brain_publisher_ = self.create_publisher(String, brain_input_topic, 10)

        # 創建發布者，將控制命令發送到vision_module
        self.vision_publisher_ = self.create_publisher(String, vision_input_topic, 10)

        # 創建發布者，將記憶存儲請求發送到memory_module
        self.memory_store_publisher_ = self.create_publisher(String, store_memory_topic, 10)

        # 創建發布者，將記憶檢索請求發送到memory_module
        self.memory_retrieve_publisher_ = self.create_publisher(String, retrieve_memory_topic, 10)

        # 設置一個計時器，每隔2秒觸發一次
        self.timer = self.create_timer(1.0, self.life_cycle)

        self.desktop_world_service_name = 'register_to_desktop_world'

        self.register_to_desktop_world_client = self.create_client(Register, self.desktop_world_service_name)

        # 註冊到世界節點
        self.register_robot_to_world()

        # 動態訂閱世界狀態
        self.dynamic_subscriber = None

        # 初始化狀態
        self.state = "IDLE"
        self.get_logger().info(f'{self.robot_name} RobotNode is up and running with high-frequency timer...')

    def register_robot_to_world(self):
        # 向世界節點註冊機器人
        request = Register.Request()
        request.robot_name = self.robot_name
        self.register_to_desktop_world_client.wait_for_service()
        future = self.register_to_desktop_world_client.call_async(request)
        future.add_done_callback(self.registration_callback)

    def registration_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Registration response: {response.message}')
        
        # 註冊成功後，訂閱世界狀態話題
        if response.success:
            topic_name = f'/desktop_world/{self.robot_name}/world_state'
            self.dynamic_subscriber = self.create_subscription(RosImage, topic_name, self.dynamic_subscription_callback, 10)
            self.get_logger().info(f'Subscribed to topic: {topic_name}')
        else:
            self.get_logger().error(f"Failed to register: {response.message}")

    def dynamic_subscription_callback(self, msg):
        # 處理收到的動態訂閱消息
        self.get_logger().info(f'Received world state update')

    def life_cycle(self):
        """
        模擬機器人持續運行的生命週期，在這裡決定何時與其他模組交互。
        """

        if self.state == "IDLE":
            self.decide_next_action()
        elif self.state == "EXPLORE":
            self.request_vision_input()
        elif self.state == "THINK":
            self.request_brain_thought()
        elif self.state == "REMEMBER":
            self.retrieve_memory()

    def decide_next_action(self):
        # 模擬隨機選擇下一個動作
        # next_action = random.choice(["EXPLORE", "THINK", "REMEMBER", "IDLE"])
        # self.get_logger().info(f'Decided next action: {next_action}')
        # self.state = next_action
        self.state = "IDLE"

    def request_vision_input(self):
        # 發送消息到vision_module來獲取圖像處理結果
        vision_msg = String()
        vision_msg.data = 'Capture and analyze image'
        self.vision_publisher_.publish(vision_msg)
        self.get_logger().info(f'Sent message to vision_module: {vision_msg.data}')
        # 更改狀態
        self.state = "IDLE"

    def request_brain_thought(self):
        # 發送消息到brain_module來請求進行思考
        brain_msg = String()
        #brain_msg.data = 'What is your next move?'
        
        final_prompt = str(self.memory) + ", 以上是你的記憶，記憶是過去的事情。你現在是一隻狗，並且正在思考與自言自語，同時你可能正在進行一些行動，你說的話會是下一步的行動，請自行發揮想像力世界的變化，並且有時間的推進。"
        final_prompt += "簡短回應你的行動，說話的對象是你自己。"

        brain_msg.data = final_prompt

        
        self.brain_publisher_.publish(brain_msg)
        self.get_logger().info(f'Sent message to brain_module: {brain_msg.data}')

        # 更改狀態
        self.state = "IDLE"

    def store_memory(self, memory_content):
        # 發送存儲記憶的請求到memory_module
        store_msg = String()
        store_msg.data = memory_content
        self.memory_store_publisher_.publish(store_msg)
        self.get_logger().info(f'Sent store memory request: {memory_content}')

    def retrieve_memory(self):
        # 發送檢索記憶的請求到memory_module
        retrieve_msg = String()
        retrieve_msg.data = ''  # 檢索請求不需要內容
        self.memory_retrieve_publisher_.publish(retrieve_msg)
        self.get_logger().info('Sent retrieve memory request.')
        # 更改狀態
        self.state = "IDLE"

    def brain_callback(self, msg):
        self.get_logger().info(f'Received from brain_module: {msg.data}')
        # 根據收到的消息決定下一個動作（可擴展）
        
        memory = {
            "content": msg.data,
            "timestamp": time.time()
        }

        # 存儲思考結果到記憶中
        self.store_memory(str(memory))

    def vision_callback(self, msg):
        self.get_logger().info(f'Received from vision_module: {msg.data}')
        # 根據收到的消息決定下一個動作（可擴展）

    def memory_callback(self, msg):
        self.get_logger().info(f'Received memory data: {msg.data}')
        self.memory = msg.data
        # 根據檢索到的記憶進行下一步動作（可擴展）

def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)