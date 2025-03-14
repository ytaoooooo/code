import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from queue import Queue
import random

class NovelPublisher(Node):
    def __init__(self):
        super().__init__('novel_publisher')
        # 创建一个发布者，发布String类型的消息到novel_topic话题，队列大小为10
        # （队列大小指的是在网络传输过程中，消息队列中可以存储的最大消息数量
        self.publisher_ = self.create_publisher(String, 'novel_topic', 10)
        self.novel_queue = Queue(maxsize=10)
        timer_period = 2  # 定时器周期为2秒
        # 创建一个定时器，每隔timer_period秒调用一次timer_callback函数
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.download_novel()
        if self.novel_queue.qsize() > 0:
            text = self.novel_queue.get()
            # 创建一个String类型的消息
            msg = String()
            # 设置消息的数据字段
            msg.data = text
            # 发布消息
            self.publisher_.publish(msg)
            # 记录日志，显示发布的消息
            self.get_logger().info('Publishing: "%s"' % msg.data)

    def download_novel(self):
        sentences = [
            "今天的天气真是晴朗，阳光明媚。",
            "人工智能技术正在快速发展，改变着我们的生活方式。",
            "读书是获取知识的重要途径，每天坚持阅读很有益处。",
            "健康的生活方式包括均衡饮食和适当运动。",
            "探索宇宙的奥秘是人类永恒的追求。",
            "音乐能够陶冶情操，给人带来愉悦的心情。",
            "环保意识对于保护地球环境至关重要。",
            "学习一门新语言可以开拓视野，增进文化理解。",
            "科技创新推动着社会不断向前发展。",
            "友谊是人生中最珍贵的财富之一。"
        ]
        # 随机选择1-3个句子组成段落
        num_sentences = random.randint(1, 3)
        selected_sentences = random.sample(sentences, num_sentences)
        text = " ".join(selected_sentences)
        # 随机决定是否添加额外的消息到队列
        extra_messages = random.randint(0, 3)  # 随机添加0-3条额外消息
        
        # for _ in range(extra_messages):
        #     # 再次随机选择1-2个句子组成新段落
        #     extra_num_sentences = random.randint(1, 2)
        #     extra_selected_sentences = random.sample(sentences, extra_num_sentences)
        #     extra_text = " ".join(extra_selected_sentences)
            
            # # 检查队列是否已满
            # if not self.novel_queue.full():
            #     self.novel_queue.put(extra_text)
            #     self.get_logger().info(f'添加额外消息到队列: "{extra_text}"')
            # else:
            #     self.get_logger().warn('队列已满，无法添加更多消息')
            #     break
        self.novel_queue.put(text)




def main(args=None):
    rclpy.init(args=args)
    novel_publisher = NovelPublisher()
    rclpy.spin(novel_publisher)
    novel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
