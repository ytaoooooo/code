import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        # 调用父类构造函数并传递节点名称
        super().__init__('minimal_node')
        # 使用日志记录器输出节点启动信息
        self.get_logger().info('ROS2 Python节点已启动。')
        self.get_logger().warn('ROS2 Python节点已启动。')

def main(args=None):
    # 初始化rclpy库
    rclpy.init(args=args)
    # 创建MinimalNode实例
    node = MinimalNode()
    # 保持节点活动状态，等待并处理ROS2消息
    rclpy.spin(node)
    # 销毁节点
    node.destroy_node()
    # 关闭rclpy库
    rclpy.shutdown()

if __name__ == '__main__':
    # 只有在直接运行脚本时才会调用main函数
    main()