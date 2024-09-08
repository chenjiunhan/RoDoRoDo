import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch  # PyTorch library, for YOLO
from ultralytics import YOLO
import os

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # 初始化 CvBridge 物件
        self.bridge = CvBridge()

        model_path = os.path.join(os.path.dirname(__file__), "..", "model", 'yolov8l.pt')
        # 載入 YOLO 模型
        self.model = YOLO(model_path)
        # 訂閱影像主題
        self.image_subscription = self.create_subscription(
            RosImage,
            '/vision/YOLO8_input',  # 替換為你實際使用的影像主題
            self.image_callback,
            10)

        # 建立一個發布者來發布偵測結果
        self.detection_image_publisher = self.create_publisher(RosImage, '/vision/YOLO8_output_image', 10)

        self.get_logger().info('Object Detector Node has been started.')

    def image_callback(self, msg):
        self.get_logger().info('Received an image.')
        # 使用 CvBridge 將 ROS Image 轉換為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 將 BGR 轉換為 RGB，因為 PyTorch 模型預期輸入為 RGB 格式
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # 執行物件偵測
        results = self.model.predict(rgb_image, verbose=False)

        annotated_frame = results[0].plot()

        # 將 RGB 轉換回 BGR，因為 ROS 需要 BGR 格式
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)

        # 將 OpenCV 格式的影像轉換為 ROS Image
        ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        # 發布處理後的影像
        self.detection_image_publisher.publish(ros_image)

        # for result in results:
        #     boxes = result.boxes  # Boxes object for bounding box outputs
        #     masks = result.masks  # Masks object for segmentation masks outputs
        #     keypoints = result.keypoints  # Keypoints object for pose outputs
        #     probs = result.probs  # Probs object for classification outputs
        #     obb = result.obb  # Oriented boxes object for OBB outputs
            #result.show()  # display to screen
            #result.save(filename="result.jpg")  # save to disk

        # 發布偵測結果圖片
        

        # # 從結果中提取物件和位置
        # detections = results.xyxy[0]  # 取出第一個batch的偵測結果
        # detection_info = []
        # for detection in detections:
        #     x1, y1, x2, y2, conf, cls = detection
        #     object_info = f"物件類別：{self.model.names[int(cls)]}, 位置：({x1}, {y1}), ({x2}, {y2}), 置信度：{conf}"
        #     detection_info.append(object_info)
        #     self.get_logger().info(object_info)

        # 發布偵測結果
        # detection_result_msg = String()
        # detection_result_msg.data = '\n'.join(detection_info)
        # self.detection_publisher.publish(detection_result_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()