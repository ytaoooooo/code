ROS2学习

# echo $AMENT_PREFIX_PATH
ament是ROS 2中用于构建、测试和安装软件包的工具集，它提供了一套统一的构建系统和工作流程，旨在简化ROS 2软件包的开发和管理。

# .bashrc 
是 Bash shell 的启动脚本之一，全称是 Bourne Again Shell Resource Configuration，它用于配置用户的 shell 环境。每当你打开一个新的 交互式、非登录 的 shell（例如在终端中打开一个新的 tab 或窗口），Bash 就会读取 .bashrc 并执行其中的命令。

# setup.bash
    让终端能够找到当前 工作空间的执行文件和库 依赖等
    

# ROS2底层通信使用的是DDS
    所以只有话题 订阅发布通信
    服务通信由两个话题
    参数通信由多个服务+通信
    动作通信由话题+服务

# 话题
    话题是一种 发布/订阅（Publish-Subscribe） 机制，适用于 连续数据流 的通信。例如，传感器数据、机器人状态等通常使用话题进行传输。
# 参数
    参数用于存储和管理 可配置的变量，可以在运行时更改，而不需要修改代码。例如，控制器的增益、传感器的阈值、更新频率等

# 服务
    服务是一种 请求-响应（Request-Response） 机制，适用于 一次性请求 而不是连续流数据。比如启动/停止机器人、获取当前位姿等。

# 动作
    动作是一种 异步长时间任务处理 机制，适用于 可能需要一段时间完成的任务，如 导航、抓取、移动 等。

# TF坐标变换
   ## 旋转坐标转换库 
    sudo apt install ros-$ROS_DISTRO-tf-transformations