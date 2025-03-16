import rclpy
from rclpy.node import Node
import tf2_ros
import math


class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer_ = tf2_ros.Buffer()
        # 创建坐标变换监听器
        self.listener = tf2_ros.TransformListener(self.buffer_, self)
        # 创建定时器
        self.timer = self.create_timer(1, self.tf_listener_callback)

    # 作用
    def tf_listener_callback(self):
        try:
            result = self.buffer_.lookup_transform('base_link', 'bottle_link', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1.0))
            transform = result.transform
            self.get_logger().info(f'坐标变换消息transform: {transform.translation}')
            self.get_logger().info(f'坐标变换消息rotation: {transform.rotation}')
            pass

        except Exception as e:
            self.get_logger().error(f'坐标变换消息获取失败: {e}')


def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    # 创建动态坐标变换发布者
    tf_listener_node = TFListener()
    # 运行节点
    rclpy.spin(tf_listener_node)
    # 销毁节点
    tf_listener_node.destroy_node()
    # 关闭ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()  