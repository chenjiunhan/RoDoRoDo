import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge  # 用於ROS和OpenCV之間轉換
import cv2

class ObjectDetectionVisualizerNode(Node):
    def __init__(self):
        super().__init__('vision_visualizer_node')

        # 創建CvBridge對象
        self.bridge = CvBridge()

        # 訂閱來自vision_module的圖像數據
        self.vision_subscription = self.create_subscription(
            RosImage,
            '/vision/YOLO8_output_image',  # 替換成你的實際主題名稱
            self.vision_callback,
            10)
        
        # 初始化窗口
        cv2.namedWindow("Vision Module Image", cv2.WINDOW_NORMAL)
        # 設置窗口位置 (x, y)
        width = 400
        height = 300
        base_y = 30
        start_x = 0
        start_y = base_y + height * 1
        cv2.moveWindow("Vision Module Image", start_x, start_y)
        cv2.resizeWindow("Vision Module Image", width, height)

        self.get_logger().info('Subscribed to /vision/YOLO8_output_image topic')

    def vision_callback(self, msg):
        # 使用CvBridge將ROS Image消息轉換為OpenCV格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 使用OpenCV顯示圖片
        cv2.imshow("Vision Module Image", cv_image)
        cv2.waitKey(1)  # 不阻塞主執行緒

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # 關閉所有OpenCV窗口

if __name__ == '__main__':
    import sys
    main(sys.argv)