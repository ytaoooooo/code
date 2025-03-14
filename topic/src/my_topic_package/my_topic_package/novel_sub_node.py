import rclpy
import espeakng
from rclpy.node import Node
from std_msgs.msg import String 

from queue import Queue
import threading
import time


class NovelSubscriber(Node):
    def __init__(self):
        super().__init__('novel_subscriber')
        # 创建一个订阅者,订阅novel_topic话题,消息类型为String
        self.novels_queue = Queue()
        # 当收到消息时会调用listener_callback回调函数
        self.subscription = self.create_subscription(
            String,
            'novel_topic', 
            self.listener_callback,
            10)

        self.speech_thread = threading.Thread(target=self.speech_callback)
        self.speech_thread.start()
        # 防止subscription被垃圾回收
        # 这里不需要赋值,因为在create_subscription时已经完成了订阅者的创建和初始化
        # 这行代码只是为了消除IDE的警告,表明我们确实需要这个subscription对象
        self.subscription  

    def listener_callback(self, msg):
        self.novels_queue.put(msg.data)
        # 收到消息后打印日志
        self.get_logger().info('I heard: "%s"' % msg.data)

    def speech_callback(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'zh'
        

        while rclpy.ok():
            if not self.novels_queue.empty():
                text = self.novels_queue.get()
                self.get_logger().info('Speaking: "%s"' % text)
                speaker.say(text)


                speaker.wait()
            else:
                # 不用的时候休眠
                time.sleep(1)

def main(args=None):
    # 初始化ROS2 Python接口
    rclpy.init(args=args)
    
    # 创建订阅者节点
    novel_subscriber = NovelSubscriber()
    
    # 保持节点运行,直到收到终止信号
    rclpy.spin(novel_subscriber)
    
    # 销毁节点并关闭ROS2接口
    novel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
