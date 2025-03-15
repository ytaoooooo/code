import rclpy
from rclpy.node import Node
from interfaces.srv import FaceDetector
import cv2
from cv_bridge import CvBridge
import os
import time
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType

class FaceDetectClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.bridge = CvBridge()
        # 创建客户端 参数：服务类型，服务名称
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource', 'bus.jpg')
        self.future = None
        self.image = None

    def update_detect_up_number(self,number):
        print('test')
        # 创建参数
        parameter = Parameter()
        parameter_value = ParameterValue()
        parameter_value.integer_value = number
        parameter_value.type = ParameterType.PARAMETER_INTEGER
        parameter.value = parameter_value
        parameter.name = 'number_of_times_to_upsample'
        
        parameters = [parameter]
        print('test')
        # 调用参数服务器修改参数
        response = self.call_set_parameters(parameters)
        for result in response.results:
            if result.successful:
                self.get_logger().info(f"参数更新成功: {result.reason}")
            else:
                self.get_logger().error(f"参数更新失败: {result.reason}")
        

    # 调用参数服务器修改参数
    def call_set_parameters(self,parameters):
        # 创建一个客户端 等待参数服务器启动 参数：服务类型，服务名称
        self.set_parameters_client = self.create_client(SetParameters, '/face_detect_node/set_parameters')
        # 等待参数服务器启动
        while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_parameters service not available, waiting again...')
        # 创建request
        request = SetParameters.Request()
        # 创建参数
        request.parameters = parameters
        # 调用服务端更新参数
        future = self.set_parameters_client.call_async(request)

        # 等待future返回结果
        rclpy.spin_until_future_complete(self, future)

        # 获取返回结果
        response = future.result()
        return response
        


        

    def send_request(self):
        # 等待服务启动
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = FaceDetector.Request()
        image = cv2.imread(self.image_path)
        self.image = image
        # 将图片转换为ros2的图片格式
        request.image = self.bridge.cv2_to_imgmsg(image)
        # 发送请求 异步 future现在并没有返回结果
        self.future = self.client.call_async(request)
        # 等待服务器返回结果后，回调函数show_response
        self.future.add_done_callback(self.show_response)
        
        # 参数1：当前节点 参数2：future对象 会阻塞等待future返回结果
        rclpy.spin_until_future_complete(self, self.future)
        # 获取返回结果
        response = self.future.result()
        self.get_logger().info(f"接收到服务器返回结果， 共有{response.number}个人脸,耗时{response.use_time}秒")
        # 显示人脸
        self.show_response(response)

    def show_response(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            # 参数: 图片, 左上角坐标, 右下角坐标, 颜色, 线宽
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 255), 4)
        # 显示图片
        cv2.imshow('face detect', self.image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    face_detect_client = FaceDetectClient()

    # 更新参数
    face_detect_client.update_detect_up_number(2)
    face_detect_client.send_request()


    # 循环等待
    rclpy.spin(face_detect_client)
    
    # 关闭rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()