import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MemoryNode(Node):
    def __init__(self):
        super().__init__('memory_node')

        # 聲明機器人名字參數
        self.declare_parameter('robot_name', 'default_robot')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # 動態設置存儲和檢索記憶的主題名稱
        store_memory_topic = f'/{self.robot_name}/store_memory'
        retrieve_memory_topic = f'/{self.robot_name}/retrieve_memory'
        memory_output_topic = f'/{self.robot_name}/memory_output'

        # 創建一個列表來存儲機器人的記憶
        self.memory = []

        # 訂閱記憶存儲和檢索的主題
        self.store_memory_subscription = self.create_subscription(
            String,
            store_memory_topic,
            self.store_memory_callback,
            10)

        self.retrieve_memory_subscription = self.create_subscription(
            String,
            retrieve_memory_topic,
            self.retrieve_memory_callback,
            10)

        # 創建發布者，將檢索的記憶信息發布出去
        self.memory_publisher_ = self.create_publisher(String, memory_output_topic, 10)

        self.get_logger().info(f'{self.robot_name} MemoryNode is up and running...')

    def store_memory_callback(self, msg):
        """
        存儲記憶的回調函數。消息格式應該是 "memory_content"。
        """
        memory_content = msg.data
        self.memory.append(memory_content)  # 將記憶添加到列表中
        self.get_logger().info(f'Stored memory for {self.robot_name}: {memory_content}')

    def retrieve_memory_callback(self, msg):
        """
        檢索記憶的回調函數。消息格式應該是空消息，因為已經綁定到機器人名字。
        """
        if self.memory:
            memories = ', '.join(self.memory)
            self.get_logger().info(f'Retrieving memory for {self.robot_name}: {memories}')
            # 發布檢索到的記憶
            output_msg = String()
            output_msg.data = f'{self.robot_name}: {memories}'
            self.memory_publisher_.publish(output_msg)
        else:
            self.get_logger().info(f'No memory found for {self.robot_name}')
            output_msg = String()
            output_msg.data = f'No memory found for {self.robot_name}'
            self.memory_publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    memory_node = MemoryNode()
    rclpy.spin(memory_node)
    memory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)