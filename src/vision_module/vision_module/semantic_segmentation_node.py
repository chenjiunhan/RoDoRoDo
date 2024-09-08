import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch  # PyTorch library, for YOLO
from ultralytics import YOLO
import os
from PIL import Image
from torchvision import transforms
import numpy as np
from segment_anything import build_sam_vit_b, SamAutomaticMaskGenerator

class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__('semantic_segmentation_node')

        # 初始化 CvBridge 物件
        self.bridge = CvBridge()

        
        self.get_logger().info('Loading DeepLabV3 model...')
        # 載入模型

        self.model_name = "sam_vit_b_01ec64"

        if self.model_name == "sam_vit_b_01ec64":
            from huggingface_hub import hf_hub_download
            # model_dir = os.path.join(os.path.dirname(__file__), "..", "model")
            chkpt_path = hf_hub_download("ybelkada/segment-anything", "checkpoints/sam_vit_b_01ec64.pth")

            self.model = SamAutomaticMaskGenerator(build_sam_vit_b(checkpoint=chkpt_path).to("cuda"))

        elif self.model_name == "deeplabv3":
            self.model = torch.hub.load('pytorch/vision:v0.10.0', 'deeplabv3_resnet101', pretrained=True)
            self.model.eval()
        
            self.preprocess = transforms.Compose([
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ])
            if torch.cuda.is_available():
                self.model.to('cuda')

        # 訂閱影像主題
        self.image_subscription = self.create_subscription(
            RosImage,
            '/vision/semantic_segmentation_input',  # 替換為你實際使用的影像主題
            self.image_callback,
            10)

        # 建立一個發布者來發布偵測結果
        self.detection_image_publisher = self.create_publisher(RosImage, '/vision/semantic_segmentation_output_image', 10)

        self.get_logger().info('Semantic Segmentation Node has been started.')

    def deeplabv3_predict(self, image):
        input_tensor = self.preprocess(image)
        input_batch = input_tensor.unsqueeze(0)

        if torch.cuda.is_available():
            input_batch = input_batch.to('cuda')

        # 執行物件偵測
        #results = self.model.predict(cv_image)

        with torch.no_grad():
            output = self.model(input_batch)['out'][0]
            output_predictions = output.argmax(0).cpu().numpy()

        num_classes = 21  # 假設有21個類別
        hsv_colors = [(i * 180 // num_classes, 255, 255) for i in range(num_classes)]
        palette = np.array([cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_HSV2RGB)[0][0] for color in hsv_colors], dtype=np.uint8)

        # 創建彩色結果圖像
        annotated_frame = palette[output_predictions]

        # 將 RGB 轉換回 BGR，因為 ROS 需要 BGR 格式
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)

        return annotated_frame
    
    def sam_vit_b_01ec64_predict(self, image):
        # 生成遮罩
        masks = self.model.generate(image)
       
        # 初始化一個空的圖像來存儲最終結果
        result_image = np.array(image).copy()
        
        # 創建一個固定的調色盤來為每個遮罩分配顏色
        num_classes = len(masks)
        hsv_colors = [(i * 180 // num_classes, 255, 255) for i in range(num_classes)]
        palette = np.array([cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_HSV2RGB)[0][0] for color in hsv_colors], dtype=np.uint8)
        
        # 遍歷每個生成的遮罩
        for i, mask_info in enumerate(masks):
            mask = mask_info['segmentation']  # Mask 是一個二值遮罩（0 和 1）
            
            # 將二值遮罩轉換為彩色遮罩
            colored_mask = np.zeros_like(result_image)
            colored_mask[mask] = palette[i]  # 將遮罩區域設置為調色盤中的顏色
            
            # 疊加彩色遮罩到原始圖像上
            result_image = cv2.addWeighted(result_image, 1.0, colored_mask, 0.5, 0)

        # 將結果圖像轉換為 BGR 格式（ROS 使用的格式）
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)

        return result_image
     

    def image_callback(self, msg):
        self.get_logger().info('Received an image.')
        # 使用 CvBridge 將 ROS Image 轉換為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 將 BGR 轉換為 RGB，因為 PyTorch 模型預期輸入為 RGB 格式
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        if self.model_name == "sam_vit_b_01ec64":
            annotated_frame = self.sam_vit_b_01ec64_predict(rgb_image)
        else:
            annotated_frame = self.deeplabv3_predict(rgb_image)

        # 將 OpenCV 格式的影像轉換為 ROS Image
        ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        # 發布處理後的影像
        self.detection_image_publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()