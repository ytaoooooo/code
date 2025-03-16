import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        # 创建动态坐标变换发布者
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1, self.publish_dynamic_tf)

    def publish_dynamic_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'bottle_link'
        # 设置坐标变换消息的参数
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.5
        # 欧拉角转换为四元数 x y z w
        quaternion = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        # 发布坐标变换消息
        self.dynamic_tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f'发布坐标变换消息transform: {transform}')


def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    # 创建动态坐标变换发布者
    dynamic_tf_broadcaster = DynamicTFBroadcaster()
    # 运行节点
    rclpy.spin(dynamic_tf_broadcaster)
    # 销毁节点
    dynamic_tf_broadcaster.destroy_node()
    # 关闭ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()  