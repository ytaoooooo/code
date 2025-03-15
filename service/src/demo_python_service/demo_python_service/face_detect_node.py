import rclpy
from rclpy.node import Node
from interfaces.srv import FaceDetector
import cv2
import face_recognition
from ament_index_python.packages import get_package_share_directory
import os   
from cv_bridge import CvBridge
import time

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.get_logger().info("Face detect node initialized")
        # 客户端请求服务 服务端处理请求
        # 客户端发送图片 服务端处理图片 返回人脸位置
        # 创建服务 参数：服务类型，服务名称，回调函数  
        self.srv = self.create_service(FaceDetector, 'face_detect', self.face_detect_callback)

        # 创建CvBridge
        self.bridge = CvBridge()
        # 上采样次数
        self.number_of_times_to_upsample = 1
        # 模型
        self.model = "hog"
        # 图片路径
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource', 'default.jpg')
    # 回调函数
    def face_detect_callback(self, request, response):
        

        if request.image.data:
            print("request.image.data: ", request.image.data)
            # 如果图片不为空，则读取图片
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            self.get_logger().error("Image is None")
            # 如果图片为空，则读取默认图片
            cv_image = cv2.imread(self.default_image_path)
        
        # 开始时间
        start_time = time.time()
        self.get_logger().info("Start face detect")
        # 检测人脸
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.number_of_times_to_upsample, model=self.model)
        # 人脸检测时间
        response.use_time = time.time() - start_time
        # 打印日志
        self.get_logger().info(f"Face detect time: {response.use_time}")


        # 获取人脸数量
        response.number = len(face_locations)
        # 将人脸位置添加到响应中
        for face_location in face_locations:
            top, right, bottom, left = face_location
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        
        # 必须返回response
        return response

def main(args=None):
    rclpy.init(args=args)
    face_detect_node = FaceDetectNode()
    rclpy.spin(face_detect_node)
    face_detect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()