#include "rclcpp/rclcpp.hpp"
#include <iostream>
// 定义一个继承自 rclcpp::Node 的类
class MinimalNode : public rclcpp::Node
{
public:
    // 节点的构造函数
    MinimalNode() : Node("minimal_node")
    {
        // 当节点创建时记录一条消息
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2!");
    }
};

int main(int argc, char *argv[])
{
    // 输出参数数量和程序名字
    std::cout << "Number of arguments: " << argc << std::endl;
    std::cout << "Program name: " << argv[0] << std::endl;

    // 初始化 ROS 2 系统
    rclcpp::init(argc, argv);
    
    // 创建一个节点对象  智能指针不需要手动释放
    auto node = std::make_shared<MinimalNode>();

    // 保持节点运行，直到接收到退出信号
    rclcpp::spin(node);

    // 关闭 ROS 2 系统
    rclcpp::shutdown();
    return 0;
}