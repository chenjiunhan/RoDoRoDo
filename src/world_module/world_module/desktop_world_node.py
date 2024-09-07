# world_module/desktop_world_node.py
import pyautogui
import rclpy
import numpy as np
import cv2
from PIL import Image, ImageDraw
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage  # 使用ROS Image消息類型
from cv_bridge import CvBridge  # 使用cv_bridge來轉換OpenCV和ROS圖像格式
from common_interface.srv import Register
from std_msgs.msg import String

class DesktopWorldNode(Node):
    def __init__(self):
        super().__init__('desktop_world_node')
        #self.publisher = self.create_publisher(RosImage, '/desktop_world/desktop_state', 10)

        # 儲存機器人的狀態話題
        self.robot_state_publishers = {}  # {robot_name: publisher}
        
        # 創建服務來允許機器人註冊
        self.create_service(Register, 'register_to_desktop_world', self.handle_register_request)

        self.FPS = 60

        self.timer = self.create_timer(1.0 / self.FPS, self.publish_desktop_state)
        self.get_logger().info('DesktopWorldNode has been started.')

        # 設置截圖區域
        sidebar_width = 80
        screen_width, screen_height = pyautogui.size()
        start_x, start_y = 1246, 92
        width = 420
        height = 314
        capture_region = (start_x, start_y, width, height)
        self._capture_region = capture_region

        # 初始化cv_bridge
        self.bridge = CvBridge()

        self.default_publisher = self.create_publisher(RosImage, '/desktop_world/default/world_state', 10)

    def publish_desktop_state(self):
        """
        模擬桌面世界中的狀態更新（如應用程序打開或關閉）。
        """
        # 截取屏幕
        screenshot = pyautogui.screenshot(region=self._capture_region)
        
        # 取得滑鼠座標
        mouse_x, mouse_y = pyautogui.position()
        relative_mouse_x = mouse_x - self._capture_region[0]
        relative_mouse_y = mouse_y - self._capture_region[1]

        # 在截圖上畫滑鼠指標
        screenshot_with_mouse = self.draw_mouse_cursor(screenshot, relative_mouse_x, relative_mouse_y)
        screenshot_np = np.array(screenshot_with_mouse)
        screenshot_np = cv2.cvtColor(screenshot_np, cv2.COLOR_RGB2BGR)  # 轉換成OpenCV格式

        # 使用cv_bridge將OpenCV圖像轉換為ROS圖像消息
        ros_image = self.bridge.cv2_to_imgmsg(screenshot_np, encoding="bgr8")

        # 發布給註冊的機器人
        for publisher in self.robot_state_publishers.values():
            publisher.publish(ros_image)

        # 發布給預設機器人
        self.default_publisher.publish(ros_image)

        self.get_logger().info('Desktop state updated and image published.')

    def handle_register_request(self, request, response):
        robot_name = request.robot_name  # 使用機器人的名字作為註冊信息
        self.get_logger().info(f'Received registration request from {robot_name}.')
        
        if robot_name not in self.robot_state_publishers:
            # 為這個機器人創建專屬的話題
            topic_name = f'/desktop_world/{robot_name}/world_state'
            publisher = self.create_publisher(RosImage, topic_name, 10)
            self.robot_state_publishers[robot_name] = publisher
            self.get_logger().info(f'Created topic for {robot_name}: {topic_name}')
            response.success = True
            response.message = f"Registered successfully, subscribed to {topic_name}"
        else:
            response.success = False
            response.message = "Already registered."
        
        return response

    def draw_mouse_cursor(self, image, x, y):
        """
        在截圖中畫滑鼠指標。
        """
        draw = ImageDraw.Draw(image)
        cursor_size = 10
        draw.ellipse((x - cursor_size, y - cursor_size, x + cursor_size, y + cursor_size), fill="red")
        return image

def main(args=None):
    rclpy.init(args=args)
    node = DesktopWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()