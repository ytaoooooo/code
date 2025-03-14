#include "rclcpp/rclcpp.hpp"

class MinimalNode : public rclcpp::Node
{
public:
    MinimalNode() : Node("cpp_node")
    {
        // 打印日志信息
        RCLCPP_INFO(this->get_logger(), "Hello, World!");
    }
};

int main(int argc, char *argv[])
{
    // 初始化ROS 2客户端库
    rclcpp::init(argc, argv);
    
    // 创建节点
    // 使用共享指针管理MinimalNode实例的生命周期
    // 共享指针在C++中用于自动管理对象的内存，确保对象在不再使用时自动释放
    auto node = std::make_shared<MinimalNode>();
    
    // 运行节点: 调用节点的构造函数
    rclcpp::spin(node);
    
    // 关闭ROS 2客户端库
    rclcpp::shutdown();
    return 0;
}
