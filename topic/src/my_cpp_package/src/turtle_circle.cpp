#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtlePublisher : public rclcpp::Node
{
public:
    TurtlePublisher() : Node("turtle_publisher")
    {
        // 创建发布者,发布到/turtle1/cmd_vel话题
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        // 创建定时器,每100ms触发一次回调
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtlePublisher::timer_callback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  // 用于发布速度指令的发布者对象
    rclcpp::TimerBase::SharedPtr timer_;  // 定时器对象,用于定期触发回调函数发布消息
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        // 在这里设置要发布的速度指令
        message.linear.x = 2.0;  // 线速度
        message.angular.z = 1.0; // 角速度
        
        // 发布消息
        publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlePublisher>());
    rclcpp::shutdown();
    return 0;
}
