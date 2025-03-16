import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        # 创建静态坐标变换发布者
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'
        # 设置坐标变换消息的参数
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6
        # 欧拉角转换为四元数 x y z w
        quaternion = quaternion_from_euler(math.radians(180), 0, 0)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        # 发布坐标变换消息
        self.static_tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f'发布坐标变换消息transform: {transform}')


def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    # 创建静态坐标变换发布者
    static_tf_broadcaster = StaticTFBroadcaster()
    # 运行节点
    rclpy.spin(static_tf_broadcaster)
    # 销毁节点
    static_tf_broadcaster.destroy_node()
    # 关闭ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()  